/*
 * An example SMR program.
 *
 */
#ifndef COMPETITION
#define COMPETITION
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "componentserver.h"
#include "rhd.h"
#include "xmlio.h"
#include <sys/ioctl.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIDDLE_LINE_SENSOR 4
#define IR_SENSOR_KA 10.611269153170243
#define IR_SENSOR_KB 83.6498613968226

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
  double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *getinputref(const char *sym_name, symTableElement *tab) {
  int i;
  for (i = 0; i < getSymbolTableSize('r'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement *getoutputref(const char *sym_name, symTableElement *tab) {
  int i;
  for (i = 0; i < getSymbolTableSize('w'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}
/*****************************************
 * odometry
 */
#define WHEEL_DIAMETER 0.06522 /* m */
#define WHEEL_SEPARATION 0.26  /* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT 8000 // 24902

typedef struct {           // input signals
  int left_enc, right_enc; // encoderticks
  // parameters
  double w;      // wheel separation
  double cr, cl; // meters per encodertick
                 // output signals
  double right_pos, left_pos;
  double x, y, theta;
  // internal variables
  int left_enc_old, right_enc_old;
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);

/********************************************
 * Motion control
 */

typedef struct { // input
  int wait_time;
  int cmd;
  int curcmd;
  double speedcmd;
  double turn_speed;
  double currentspeed;
  double startangle;
  int followdirection;
  int stop_criteria;
  double dist;
  double angle; // How much to turn
  double left_pos, right_pos;
  // parameters
  double w;
  // output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
} motiontype;

enum { mot_stop = 1, mot_move, mot_turn, mot_followline, mot_followwall ,mot_turnr,mot_wait};

enum { follow_left, follow_middle, follow_right, wall_left, wall_right };

enum {
  step_1_followline_until_box = 1,
  step_2_wait,
  step_3_turn_180,
  step_3_1_followline_until_crossing,
  step_3_2_fwd,
  step_4_followline_until_right_line,
  step_5_turnr_right,
  step_6_followline_middle_until_box,
  step_7_followline_middle_until_line_stops,
  step_8_backwards,
  step_9_turn_180,
  step_10_followline_until_left_line,
  step_11_turnr_left,
  step_12_followline_until_crossing,
  step_13_turnr_left,
  step_14_followline_until_crossing,
  step_15_fwd,
  step_16_followline_until_crossing
  //CHECKPOINT2

  // step_10_followline_middle_until_gate,
  // step_11_forward_to_align_with_gate,
  // step_12_rotate_90_degrees_to_gate,
  // step_13_forward_to_enter_gate,
  // step_14_rotate_90_degrees_align_to_wall,
  // step_15_follow_wall_on_the_left,
  // step_16_forward_to_align_with_gate,
  // step_17_rotate_90_degrees_to_gate,
  // step_18_forward_to_enter_gate,
  // step_19_backwards_to_exit_gate,
  //...
};

enum {
  stop_at_box = 1,
  stop_at_lost_line,
  stop_at_crossing,
  stop_at_gate_on_the_left,
  stop_at_left_line,
  stop_at_right_line
};

void update_motcon(motiontype *p, odotype *odo);

int wait(int stop_criteria, int time, int wait_time);
int fwd(int stop_criteria, double dist, double speed, int time);
int turn(int stop_criteria, double angle, double speed, double startangle,
         int time);
int turnr(int stop_criteria, double dist, double speed,double turn_speed, int time);
int follow_line(int stop_criteria, double speed, int followdirection, int time);
int follow_wall(int stop_criteria, double speed, int followdirection, int time);

typedef struct {
  int state, oldstate;
  int time;
} smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr,
    *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {
  ms_init,
  ms_fwd,
  ms_turn,
  ms_end,
  ms_followline,
  ms_followwall,
  ms_fwd_obst,
  ms_change_step,
  ms_turnr,
  ms_wait
};

double calculate_center_of_mass(double values[], int followBlack);

double *linear_transformation(int OldValues[]);

double *normalise_ir_sensor(int rawValues[]);

int find_lowest_line_idx(double NewValues[], int followdirection);

double check_obstacle_distance();

int main() {
  int running, arg, time = 0, stop_criteria = 0, current_step = 0,wait_time=0;
  double dist = 0, angle = 0, followdirection = follow_middle, speed = 0,turn_speed=0;

  /* Establish connection to robot sensors and actuators.
   */
  if (rhdConnect('w', "localhost", ROBOTPORT) != 'w') {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }

  printf("connected to robot \n");
  if ((inputtable = getSymbolTable('r')) == NULL) {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  if ((outputtable = getSymbolTable('w')) == NULL) {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  // connect to robot I/O variables
  lenc = getinputref("encl", inputtable);
  renc = getinputref("encr", inputtable);
  linesensor = getinputref("linesensor", inputtable);
  irsensor = getinputref("irsensor", inputtable);

  speedl = getoutputref("speedl", outputtable);
  speedr = getoutputref("speedr", outputtable);
  resetmotorr = getoutputref("resetmotorr", outputtable);
  resetmotorl = getoutputref("resetmotorl", outputtable);
  // **************************************************
  //  Camera server code initialization
  //

  /* Create endpoint */
  lmssrv.port = 24919;
  strcpy(lmssrv.host, "127.0.0.1");
  strcpy(lmssrv.name, "laserserver");
  lmssrv.status = 1;
  camsrv.port = 24920;
  strcpy(camsrv.host, "127.0.0.1");
  camsrv.config = 1;
  strcpy(camsrv.name, "cameraserver");
  camsrv.status = 1;

  if (camsrv.config) {
    int errno = 0;
    camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (camsrv.sockfd < 0) {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&camsrv);

    xmldata = xml_in_init(4096, 32);
    printf(" camera server xml initialized \n");
  }

  // **************************************************
  //  LMS server code initialization
  //

  /* Create endpoint */
  lmssrv.config = 1;
  if (lmssrv.config) {
    char buf[256];
    int errno = 0, len;
    lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lmssrv.sockfd < 0) {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&lmssrv);
    if (lmssrv.connected) {
      xmllaser = xml_in_init(4096, 32);
      printf(" laserserver xml initialized \n");
      // len = sprintf(buf, "push  t=0.2 cmd='mrcobst width=0.4'\n");
      len = sprintf(buf, "scanpush cmd='zoneobst'\n");

      send(lmssrv.sockfd, buf, len, 0);
    }
  }

  // Initialise logger array
  int log_dimx = 10000;
  int log_dimy = 6;
  int log_laser_dimy = 10;
  double datalogger[log_dimx][log_dimy];
  double laserlogger[log_dimx][log_laser_dimy];
  int log_idx = 0;

  /* Read sensors and zero our position. */
  rhdSync();

  odo.w = 0.256;
  odo.cr = DELTA_M;
  odo.cl = odo.cr;
  odo.left_enc = lenc->data[0];
  odo.right_enc = renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w = odo.w;
  running = 1;
  mission.state = ms_init;
  mission.oldstate = -1;

  mot.speedcmd = 0;

  while (running) {
    if (lmssrv.config && lmssrv.status && lmssrv.connected) {
      while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
        xml_proca(xmllaser);
    }

    if (camsrv.config && camsrv.status && camsrv.connected) {
      while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
        xml_proc(xmldata);
    }

    rhdSync();
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];
    update_odo(&odo);

    // Log data
    datalogger[log_idx][0] = mission.time;
    datalogger[log_idx][1] = mot.motorspeed_l;
    datalogger[log_idx][2] = mot.motorspeed_r;
    datalogger[log_idx][3] = odo.theta;
    datalogger[log_idx][4] = odo.x;
    datalogger[log_idx][5] = odo.y;

    // laserlogger[log_idx][0] = laserpar[0];
    // laserlogger[log_idx][1] = laserpar[1];
    // laserlogger[log_idx][2] = laserpar[2];
    // laserlogger[log_idx][3] = laserpar[3];
    // laserlogger[log_idx][4] = laserpar[4];
    // laserlogger[log_idx][5] = laserpar[5];
    // laserlogger[log_idx][6] = laserpar[6];
    // laserlogger[log_idx][7] = laserpar[7];
    // laserlogger[log_idx][8] = laserpar[8];
    // laserlogger[log_idx][9] = laserpar[9];

    log_idx++;

    /************************/
    /* mission statemachine */
    /************************/

    sm_update(&mission);

    switch (mission.state) {
    case ms_init:
      // Start state
      speed = 0.4;
      followdirection = follow_right;
      current_step = step_1_followline_until_box;
      stop_criteria = stop_at_box;
      mission.state = ms_followline;

      // Test
      // speed = 0.4;
      // followdirection = follow_middle;
      // current_step = step_10_followline_middle_until_gate;
      // stop_criteria = stop_at_gate_on_the_left;
      // mission.state = ms_followline;

      // Test 2
      // speed = 0.4;
      // dist = 4;
      // angle = .5 * M_PI;
      // current_step = step_14_rotate_90_degrees_align_to_wall;
      // mission.state = ms_turn;

      break;

    case ms_change_step:
      printf("Step %d finished!\n", current_step);
      stop_criteria = 0;
      if (current_step == step_1_followline_until_box) {
        wait_time=100;
        current_step = step_2_wait;
        mission.state = ms_wait;
      } else if (current_step == step_2_wait) {
        angle = 1 * M_PI - 0.1;
        current_step = step_3_turn_180;
        mission.state = ms_turn;
      //  
      // } else if (current_step == step_3_turn_180) {
      //   followdirection = follow_middle;
      //   stop_criteria = stop_at_crossing;
      //   current_step = step_3_1_followline_until_crossing;
      //   mission.state = ms_followline;
      // } else if (current_step == step_3_1_followline_until_crossing) {
      //   speed = 0.3;
      //   dist = 0.1;
      //   current_step = step_3_2_fwd;
      //   mission.state = ms_fwd;
      // } else if (current_step == step_3_2_fwd) {
      } else if (current_step == step_3_turn_180) { //Replaced with else if from above
        followdirection = follow_middle;
        stop_criteria = stop_at_right_line;
        current_step = step_4_followline_until_right_line;
        mission.state = ms_followline;
      } else if (current_step == step_4_followline_until_right_line) {
        speed = 0.4;
        turn_speed = 0.35;
        dist = 0.25;
        current_step = step_5_turnr_right;
        mission.state = ms_turnr;
      } else if (current_step ==step_5_turnr_right) {
        followdirection = follow_middle;
        stop_criteria = stop_at_box;
        current_step = step_6_followline_middle_until_box;
        mission.state = ms_followline;
      } else if (current_step == step_6_followline_middle_until_box) {
        followdirection = follow_middle;
        stop_criteria = stop_at_lost_line;
        current_step = step_7_followline_middle_until_line_stops;
        mission.state = ms_followline;
      } else if (current_step == step_7_followline_middle_until_line_stops) {
        speed = -0.3;
        dist = 1;
        current_step = step_8_backwards;
        mission.state = ms_fwd;
      } else if (current_step == step_8_backwards) {
        speed = 0.4;
        angle = -1* M_PI +0.15;
        current_step = step_9_turn_180;
        mission.state = ms_turn;
      } else if (current_step == step_9_turn_180) {
        followdirection = follow_left;
        stop_criteria = stop_at_left_line;
        current_step = step_10_followline_until_left_line;
        mission.state = ms_followline;
      } else if (current_step == step_10_followline_until_left_line) {
        speed = 0.35;
        turn_speed = -0.25;
        dist = 0.28;
        current_step = step_11_turnr_left;
        mission.state = ms_turnr;
      } else if (current_step == step_11_turnr_left) {
        speed = 0.4;
        followdirection = follow_middle;
        stop_criteria = stop_at_crossing;
        current_step = step_12_followline_until_crossing;
        mission.state = ms_followline;
      } else if (current_step == step_12_followline_until_crossing) {
        speed = 0.3;
        turn_speed = -0.25;
        dist = 0.05;
        current_step = step_13_turnr_left;
        mission.state = ms_turnr;
      } else if (current_step == step_13_turnr_left) {
        speed = 0.4;
        followdirection = follow_middle;
        stop_criteria = stop_at_crossing;
        current_step = step_14_followline_until_crossing;
        mission.state = ms_followline;
      } else if (current_step == step_14_followline_until_crossing) {
        speed = 0.3;
        dist = 0.1;
        current_step = step_15_fwd;
        mission.state = ms_fwd;
      } else if (current_step == step_15_fwd) {
        speed = 0.4;
        followdirection = follow_middle;
        stop_criteria = stop_at_crossing;
        current_step = step_16_followline_until_crossing;
        mission.state = ms_followline;
      // } else if (current_step == step_16_followline_until_crossing) {
      
      // } else if (current_step == step_10_followline_middle_until_gate) {
      //   dist = 0.65;
      //   current_step = step_11_forward_to_align_with_gate;
      //   mission.state = ms_fwd;
      // } else if (current_step == step_11_forward_to_align_with_gate) {
      //   angle = 0.5 * M_PI;
      //   current_step = step_12_rotate_90_degrees_to_gate;
      //   mission.state = ms_turn;

      // } else if (current_step == step_12_rotate_90_degrees_to_gate) {
      //   dist = 4;
      //   stop_criteria = stop_at_box;
      //   current_step = step_13_forward_to_enter_gate;
      //   mission.state = ms_fwd;
      // } else if (current_step == step_13_forward_to_enter_gate) {
      //   angle = 0.5 * M_PI;
      //   current_step = step_14_rotate_90_degrees_align_to_wall;
      //   mission.state = ms_turn;
      // } else if (current_step == step_14_rotate_90_degrees_align_to_wall) {
      //   followdirection = wall_left;
      //   current_step = step_15_follow_wall_on_the_left;
      //   mission.state = ms_followwall;
      // } else if (current_step == step_15_follow_wall_on_the_left) {
      //   dist = 0.4;
      //   current_step = step_16_forward_to_align_with_gate;
      //   mission.state = ms_fwd;
      /*} else if (current_step == step_16_forward_to_align_with_gate) {
        angle = 0.5 * M_PI;
        current_step = step_17_rotate_90_degrees_to_gate;
        mission.state = ms_turn;

        /*} else if (current_step == step_17_rotate_90_degrees_to_gate) {
          current_step = step_18_forward_to_enter_gate;

        } else if (current_step == step_18_forward_to_enter_gate) {
          current_step = step_19_backwards_to_exit_gate;

        } else if (current_step == step_19_backwards_to_exit_gate) {
          // current_step = step_16_forward_to_align_with_gate;
          mission.state = ms_end;*/
      } else {
        mission.state = ms_end;
      }

      break;

    case ms_fwd:
      if (fwd(stop_criteria, dist, speed, mission.time))
        mission.state = ms_change_step;
      break;

    case ms_turn:
      if (turn(stop_criteria, angle, speed, odo.theta, mission.time)) {
        mission.state = ms_change_step;
      }
      break;

    case ms_followline:
      if (follow_line(stop_criteria, speed, followdirection, mission.time)) {
        mission.state = ms_change_step;
      }
      break;

    case ms_turnr:
      if (turnr(stop_criteria, dist, speed,turn_speed, mission.time)) {
        mission.state = ms_change_step;
      }
      break;

    case ms_wait:
      if(wait(stop_criteria,mission.time,wait_time)){
        mission.state = ms_change_step;
      }
      break;

    case ms_followwall:
      if (follow_wall(stop_criteria, speed, followdirection, mission.time)) {
        mission.state = ms_change_step;
      }
      break;

    case ms_end:
      mot.cmd = mot_stop;
      running = 0;
      break;
    }
    /*  end of mission  */

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot, &odo);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;
    if (time % 100 == 0)
      // printf(" laser %f \n", laserpar[3]);
      time++;
    /* stop if keyboard is activated
     *
     */
    ioctl(0, FIONREAD, &arg);
    if (arg != 0)
      running = 0;

  } /* end of main control loop */

  /* LOGGING */
  /* Dump sensor log to file */
  printf("Dumping sensor log to file!\n");
  FILE *sensor_out;
  sensor_out = fopen("sensorlog.dat", "w");
  if (sensor_out == NULL) {
    fprintf(stderr, "error opening sensorlog.dat\n");
    exit(EXIT_FAILURE);
  }
  for (int i = 0; i < log_idx; i++) {
    fprintf(sensor_out, "%.3f %.3f %.3f %.3f %.3f %.3f \n", datalogger[i][0],
            datalogger[i][1], datalogger[i][2], datalogger[i][3],
            datalogger[i][4], datalogger[i][5]);
  }
  fclose(sensor_out);

  /* Dump laser log to file */
  printf("Dumping laser log to file!\n");
  FILE *laser_out;
  laser_out = fopen("laserlog.dat", "w");
  if (laser_out == NULL) {
    fprintf(stderr, "error opening laserlog.dat\n");
    exit(EXIT_FAILURE);
  }
  for (int i = 0; i < log_idx; i++) {
    fprintf(laser_out, "%f %f %f %f %f %f %f %f %f %f\n", laserlogger[i][0],
            laserlogger[i][1], laserlogger[i][2], laserlogger[i][3],
            laserlogger[i][4], laserlogger[i][5], laserlogger[i][6],
            laserlogger[i][7], laserlogger[i][8], laserlogger[i][9]);
  }
  fclose(laser_out);
  /* END LOGGING */

  speedl->data[0] = 0;
  speedl->updated = 1;
  speedr->data[0] = 0;
  speedr->updated = 1;
  rhdSync();
  rhdDisconnect();
  exit(0);
}

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

void reset_odo(odotype *p) {
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->x = 0;
  p->y = 0;
  p->theta = 0;
}

void update_odo(odotype *p) {
  int delta_r = p->right_enc - p->right_enc_old;
  if (delta_r > 0x8000)
    delta_r -= 0x10000;
  else if (delta_r < -0x8000)
    delta_r += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta_r * p->cr;

  int delta_l = p->left_enc - p->left_enc_old;
  if (delta_l > 0x8000)
    delta_l -= 0x10000;
  else if (delta_l < -0x8000)
    delta_l += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta_l * p->cl;

  double robot_displacement = (delta_r * p->cr + delta_l * p->cl) / 2;
  double orientation_change = (delta_r * p->cr - delta_l * p->cl) / p->w;

  p->theta += orientation_change;
  if (p->theta > M_PI) {
    p->theta -= 2 * M_PI;
  }

  p->x += robot_displacement * cos(p->theta);
  p->y += robot_displacement * sin(p->theta);
}

void update_motcon(motiontype *p, odotype *odo) {

  if (p->cmd != 0) {

    p->finished = 0;
    switch (p->cmd) {
    case mot_stop:
      p->curcmd = mot_stop;
      break;

    case mot_wait:
      p->curcmd = mot_wait;
      break;

    case mot_move:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_move;
      break;

    case mot_turnr:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_turnr;
      break;

    case mot_turn:
      if (p->angle > 0)
        p->startpos = p->right_pos;
      else
        p->startpos = p->left_pos;
      p->curcmd = mot_turn;
      break;

    case mot_followline:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_followline;
      break;

    case mot_followwall:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_followwall;
      break;
    }

    p->cmd = 0;
  }

  double max_accel = 0.5;
  double delta_time = 0.01;

  double current_accel = (p->speedcmd - p->currentspeed) / delta_time;
  if (current_accel > max_accel) {
    current_accel = max_accel;
  }

  double current_angle = odo->theta - p->startangle;
  current_angle += (current_angle > M_PI)    ? -(2 * M_PI)
                   : (current_angle < -M_PI) ? (2 * M_PI)
                                             : 0;

  double delta_velocity;

  double *normalised_sensor_data = linear_transformation(linesensor->data);

  int minLineSensorIndex =
      find_lowest_line_idx(normalised_sensor_data, p->followdirection);

  // printf("linesensor %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f;idx = % d\n ",
  //        normalised_sensor_data[0], normalised_sensor_data[1],
  //        normalised_sensor_data[2], normalised_sensor_data[3],
  //        normalised_sensor_data[4], normalised_sensor_data[5],
  //        normalised_sensor_data[6], normalised_sensor_data[7],
  //        minLineSensorIndex);

  // printf("linesensor %d %d %d %d %d %d %d %d; idx=%d\n", linesensor->data[0],
  //        linesensor->data[1], linesensor->data[2], linesensor->data[3],
  //        linesensor->data[4], linesensor->data[5], linesensor->data[6],
  //        linesensor->data[7], minLineSensorIndex);

  // printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", laserpar[0],
  //        laserpar[1], laserpar[2], laserpar[3], laserpar[4], laserpar[5],
  //        laserpar[6], laserpar[7], laserpar[8], laserpar[9]);

  double *normalised_irsensor = normalise_ir_sensor(irsensor->data);

  // printf("irsensor %.2f %.2f %.2f %.2f %.2f\n", normalised_irsensor[0],
  //        normalised_irsensor[1], normalised_irsensor[2], normalised_irsensor[3],
  //        normalised_irsensor[4]);

  double K = .4;

  double obst_distance = check_obstacle_distance();

  if (p->stop_criteria == stop_at_box && obst_distance <= 0.15) {
    p->motorspeed_r = 0;
    p->motorspeed_l = 0;
    p->finished = 1;
    return;
  }

  switch (p->curcmd) {
  case mot_stop:
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
    break;

  case mot_wait:
    p->wait_time=p->wait_time-1;
    if(p->wait_time%20==0){
          printf("Stopped at %f. X Distance %f\n", obst_distance,
           obst_distance + odo->x);
    }
    if(p->wait_time==0){
       p->finished = 1;
    }
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
    break;
  
  case mot_move:
    // printf("Robot pos %f start pos %f dist %f\n",
    //        (p->right_pos + p->left_pos) / 2, p->startpos, p->dist);

    if (fabs((p->right_pos + p->left_pos) / 2 - p->startpos)>= p->dist) {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    } else {
      p->motorspeed_l = p->speedcmd;
      p->motorspeed_r = p->speedcmd;
      // p->motorspeed_l = p->speedcmd;
      // p->motorspeed_r = p->speedcmd;
    }
    break;

  case mot_turn:
    // printf("Current %.3f target %.3f theta %.3f start %.3f\n", current_angle,
    //        p->angle, odo->theta, p->startangle);

    if (p->angle > 0) {
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;

      if (current_angle > p->angle) {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      } else {
        p->currentspeed = p->speedcmd;
        p->motorspeed_r = p->currentspeed / 2;
        p->motorspeed_l = -p->currentspeed / 2;
      }
    } else {

      if (current_angle < p->angle) {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      } else {
        p->currentspeed = p->speedcmd;
        p->motorspeed_r = -p->currentspeed / 2;
        p->motorspeed_l = p->currentspeed / 2;
      }
    }

    break;


  case mot_followline:

    if (p->stop_criteria == stop_at_crossing && minLineSensorIndex == -1) {
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;
      p->finished = 1;
      break;
    }

    if (p->stop_criteria ==stop_at_left_line && normalised_sensor_data[7]<0.2){
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;
      p->finished = 1;
      break;
    }

    if (p->stop_criteria ==stop_at_right_line && normalised_sensor_data[1]<0.2){
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;
      p->finished = 1;
      break;
    }

    if (p->stop_criteria == stop_at_lost_line) {
      if (minLineSensorIndex == -1 &&
          normalised_sensor_data[MIDDLE_LINE_SENSOR] > 0.2) {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
        break;
      }
    }
    
    if (p->stop_criteria == stop_at_gate_on_the_left && laserpar[0] > 0 &&
        laserpar[0] < 0.5) {
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;
      p->finished = 1;
      break;
    }

    if (minLineSensorIndex < MIDDLE_LINE_SENSOR) {
      delta_velocity = -K * (MIDDLE_LINE_SENSOR - minLineSensorIndex);
    } else if (minLineSensorIndex > MIDDLE_LINE_SENSOR) {
      delta_velocity = K * (minLineSensorIndex - MIDDLE_LINE_SENSOR);
    } else {
      delta_velocity = 0;
    }

    if (p->followdirection == follow_middle &&
        (normalised_sensor_data[MIDDLE_LINE_SENSOR] < 0.2 ||
         normalised_sensor_data[MIDDLE_LINE_SENSOR - 1] < 0.2 ||
         normalised_sensor_data[MIDDLE_LINE_SENSOR + 1] < 0.2)) {
      delta_velocity = 0;
    }

    p->motorspeed_r = (p->speedcmd + delta_velocity) / 2;
    p->motorspeed_l = (p->speedcmd - delta_velocity) / 2;

    break;
  
  case mot_turnr:
    if (fabs((p->right_pos + p->left_pos) / 2 - p->startpos)>= p->dist) {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
      break;
    }
    else {
      p->motorspeed_r = (p->speedcmd - p->turn_speed) / 2;
      p->motorspeed_l = (p->speedcmd + p->turn_speed) / 2;
      break;
    }

  case mot_followwall:
    // 0 and 8 are the sensors
    if (normalised_irsensor[4] > 0.3) {
      printf("Stopping following wall\n");
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;
      p->finished = 1;
      break;
    }

    delta_velocity = 10 * (0.15 - normalised_irsensor[4]);

    printf("Delta V %.3f irval %.3f\n", delta_velocity, normalised_irsensor[4]);

    p->motorspeed_r = (p->speedcmd + delta_velocity) / 2;
    p->motorspeed_l = (p->speedcmd - delta_velocity) / 2;
    break;
  }
}

double *normalise_ir_sensor(int rawValues[]) {
  static double normalised_values[5];

  for (int i = 0; i < 5; i++) {
    normalised_values[i] = IR_SENSOR_KA / (rawValues[i] - IR_SENSOR_KB);
  }

  return normalised_values;
}

double check_obstacle_distance() {
  double avgDistance = (laserpar[3] + laserpar[4] + laserpar[5]) / 3;
  if (avgDistance == 0) {
    avgDistance = 1;
  }

  // printf("%f %f %f %f\n", laserpar[3], laserpar[4], laserpar[5],
  // avgDistance);
  return avgDistance;
}

double calculate_center_of_mass(double values[], int followBlack) {
  double denum = 0;
  double num = 0;
  for (int i = 0; i < 8; i++) {
    if (followBlack == 1) {
      num += i * (1 - values[i]);
      denum += (1 - values[i]);
    } else {
      num += i * values[i];
      denum += values[i];
    }
  }
  return num / denum;
}

double *linear_transformation(int oldValues[]) {
  int oldMax = 0;
  int oldMin = 128;
  double newMin = 0;
  double newMax = 1;
  double oldRange;
  double newRange;
  static double newValues[8];

  for (int i = 0; i < 8; i++) {
    if (oldValues[i] > oldMax) {
      oldMax = oldValues[i];
    }
    if (oldValues[i] < oldMin) {
      oldMin = oldValues[i];
    }
  }
  oldRange = (oldMax - oldMin);
  newRange = newMax - newMin;

  if (oldRange < 5) {
    oldMin = 0;
    oldMax = 128;
    oldRange = (oldMax - oldMin);
  }

  for (int i = 0; i < 8; i++) {
    newValues[i] = (((oldValues[i] - oldMin) * newRange) / oldRange) + newMin;
    newValues[i] = MAX(newValues[i], 0);
    newValues[i] = MIN(newValues[i], 1);
  }

  return newValues;
}

int find_lowest_line_idx(double lineValues[], int followdirection) {
  int foundLeftLineIdx = -1;
  int foundRightLineIdx = -1;
  int foundMiddleLineIdx = -1;

  int smallestValueIndex = 0;

  // Find smallest value
  for (int i = 0; i < 8; i++) {
    if (lineValues[i] < lineValues[smallestValueIndex]) {
      smallestValueIndex = i;
    }
  }

  // Find black lines at either side
  for (int i = 0; i < 8; i++) {
    if (lineValues[i] == lineValues[smallestValueIndex]) {
      if (i == MIDDLE_LINE_SENSOR) {
        foundMiddleLineIdx = i;
      } else if (i < MIDDLE_LINE_SENSOR && foundRightLineIdx == -1) {
        foundRightLineIdx = i;
      } else if (i > MIDDLE_LINE_SENSOR) {
        foundLeftLineIdx = i;
      }
    }
  }

  // Stop at cross
  bool crossLine = true;
  for (int i = 2; i < 6; i++) {
    if (lineValues[i] != lineValues[smallestValueIndex]) {
      crossLine = false;
      break;
    }
  }

  if (crossLine) {
    return -1;
  }

  switch (followdirection) {
  case follow_middle:
    if (foundMiddleLineIdx != -1) {
      return foundMiddleLineIdx;
    }
    break;
  case follow_left:
    if (foundLeftLineIdx != -1) {
      return foundLeftLineIdx;
    }
    break;
  case follow_right:
    if (foundRightLineIdx != -1) {
      return foundRightLineIdx;
    }
    break;
  }

  return smallestValueIndex;
}

int fwd(int stop_criteria, double dist, double speed, int time) {
  if (time == 0) {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.stop_criteria = stop_criteria;
    mot.currentspeed = 0;
    mot.dist = dist;
    return 0;
  } else
    return mot.finished;
}

int wait(int stop_criteria,int time,int wait_time) {
  if (time == 0) {
    mot.cmd = mot_wait;
    mot.stop_criteria = stop_criteria;
    mot.wait_time =wait_time;
    return 0;
  } else
    return mot.finished;
}

int turn(int stop_criteria, double angle, double speed, double startangle,
         int time) {
  if (time == 0) {

    mot.cmd = mot_turn;
    mot.speedcmd = speed;
    mot.currentspeed = 0;
    mot.stop_criteria = stop_criteria;
    mot.startangle = startangle;
    mot.stop_criteria = stop_criteria;
    mot.angle = angle;
    return 0;
  } else
    return mot.finished;
}

int turnr(int stop_criteria, double dist, double speed,double turn_speed, int time) {
  if (time == 0) {

    mot.cmd = mot_turnr;
    mot.speedcmd = speed;
    mot.turn_speed = turn_speed;
    mot.currentspeed = 0;
    mot.stop_criteria = stop_criteria;
    mot.dist = dist;
    return 0;
  } else
    return mot.finished;
}

int follow_line(int stop_criteria, double speed, int followdirection,
                int time) {
  if (time == 0) {
    mot.cmd = mot_followline;
    mot.speedcmd = speed;
    mot.stop_criteria = stop_criteria;
    mot.currentspeed = 0;
    mot.followdirection = followdirection;
    return 0;
  } else
    return mot.finished;
}

int follow_wall(int stop_criteria, double speed, int followdirection,
                int time) {
  if (time == 0) {
    mot.cmd = mot_followwall;
    mot.speedcmd = speed;
    mot.stop_criteria = stop_criteria;
    mot.currentspeed = 0;
    mot.followdirection = followdirection;
    return 0;
  } else
    return mot.finished;
}

void sm_update(smtype *p) {
  if (p->state != p->oldstate) {
    p->time = 0;
    p->oldstate = p->state;
  } else {
    p->time++;
  }
}
#endif
