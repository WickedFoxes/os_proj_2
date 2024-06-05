
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

// Number of vicechlees that have entered or are about to enter the deadzone
static int deadzone_cnt = 0;

// check vehicle in deadlock_zone
int deadzone[][2] = {{2,2},{2,3},{2,4},{3,2},{3,4},{4,2},{4,3},{4,4}};
int deadzone_out[][2] = {{1,4}, {2,1}, {4,5}, {5,2}};
int deadzone_in[][2] = {{2,2}, {2,4}, {4,2}, {4,4}};
bool arr_contains(int arr[][2], int size, int row, int col){
	int check = 0;
	for(int i=0; i<size; i++){
		if(arr[i][0] == row && arr[i][1] == col){
			return true;
		}
	}
	return false;
}

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
	/* from A */ {
		/* to A */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;
	pos_next = vehicle_path[start][dest][step];
	pos_cur = vi->position;

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			return 0;
		}
	}

	bool now_deadzone = arr_contains(deadzone, 8, pos_cur.row, pos_cur.col);
	bool now_deadzone_in = arr_contains(deadzone_in, 4, pos_cur.row, pos_cur.col);
	bool now_deadzone_out = arr_contains(deadzone_out, 4, pos_cur.row, pos_cur.col);
	bool next_deadzone = arr_contains(deadzone, 8, pos_next.row, pos_next.col);
	bool next_deadzone_out = arr_contains(deadzone_out, 4, pos_next.row, pos_next.col);
	
	// if vicle enter to deadzone
	if(!now_deadzone && next_deadzone) deadzone_cnt++;
	// if vehicle out of deadzone
	else if(now_deadzone_out) deadzone_cnt--;

	/* lock next position */
	lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
	if (vi->state == VEHICLE_STATUS_READY) {
		/* start this vehicle */
		vi->state = VEHICLE_STATUS_RUNNING;
	}
	else if(deadzone_cnt >= 4){
		// 현재 deadzone이 아니며, 다음에도 deadzone이 아닌 경우
		if(!now_deadzone && !next_deadzone){
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
		}
		// 현재 deadzone이며, 다음에도 deadzone인 경우
		else if(now_deadzone && next_deadzone){
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
		}
		// 현재 deadzone을 탈출하는 경우
		else if(now_deadzone_out){
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
		}
	}
	else{
		/* release current position */
		lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);		
	} 

	/* update position */
	vi->position = pos_next;
	
	return 1;
}

void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
}

void vehicle_loop(void *_vi)
{
	int res;
	int start, dest, step;

	struct vehicle_info *vi = _vi;

	start = vi->start - 'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;

	step = 0;
	while (1) {
		/* vehicle main code */
		res = try_move(start, dest, step, vi);
		if (res == 1) {
			step++;
		}

		/* termination condition. */ 
		if (res == 0) {
			break;
		}

		/* unitstep change! */
		unitstep_changed();
	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
