
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

// Number of vicechlees that have entered or are about to enter the deadzone
static bool deadzone_check[4] = {
	true, true, true, true
};

// check vehicle in deadlock_zone
int deadzone_in[][2] = {{4,1}, {5,4}, {2,5}, {1,2}};
int deadzone_out[][2] = {{2,1}, {5,2}, {4,5}, {1,4}};

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
	int v_start = vi->start - 'A';
	int v_end = vi->dest - 'A';
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

	if(!deadzone_check[v_start]
	&& pos_cur.row == deadzone_in[v_start][0] && pos_cur.col == deadzone_in[v_start][1]){
		// deadzone으로 진입하려는데 잠긴 경우
		return 1;
	}

	/* lock next position */
	lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);

	if (vi->state == VEHICLE_STATUS_READY) {
		/* start this vehicle */
		vi->state = VEHICLE_STATUS_RUNNING;
	} else {
		/* release current position */
		lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
	}
	/* update position */
	vi->position = pos_next;

	if(deadzone_check[v_start] 
	&& pos_cur.row == deadzone_in[v_start][0] && pos_cur.col == deadzone_in[v_start][1]){
		// deadzone으로 처음 진입한 경우
		// 자신이 진입한 곳 잠금
		deadzone_check[v_start] = false;
		lock_acquire(&vi->map_locks[deadzone_in[v_start][0]][deadzone_in[v_start][1]]);
	}
	if(!deadzone_check[v_start]
	&& pos_cur.row == deadzone_in[v_end][0] && pos_cur.col == deadzone_in[v_end][1]){
		// deadzone에서 빠져나온 경우
		// 자신이 출발한 곳 잠금 해제
		deadzone_check[v_start] = false;
		lock_release(&vi->map_locks[deadzone_in[v_start][0]][deadzone_in[v_start][1]]);
	}


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
