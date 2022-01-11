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
  int cmd;
  int curcmd;
  double speedcmd;
  double currentspeed;
  double startangle;
  double finalangle;
  int followdirection;
  double dist;
  double angle;
  double left_pos, right_pos;
  // parameters
  double w;
  // output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
} motiontype;

enum { mot_stop = 1, mot_move, mot_turn, mot_control };

enum { follow_left, follow_middle, follow_right };

void update_motcon(motiontype *p, odotype *odo);

int fwd(double dist, double speed, int time);
int turn(double angle, double speed, double startangle, int time);
int control(double dist, double finalangle, double speed, int followdirection,
            int time);

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

enum { ms_init, ms_fwd, ms_turn, ms_end, ms_followline, ms_fwd_obst };

double calculate_center_of_mass(double values[], int followBlack);

double *linear_transformation(int OldValues[]);

int find_lowest_line_idx(double NewValues[], int followdirection);

int main() {
  int running, n = 0, arg, time = 0, stop_at_crossing = 0;
  double dist = 0, angle = 0, followdirection = follow_middle;

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
      n = 4;
      dist = 2;
      // angle = 90.0 / 180 * M_PI;
      angle = 0;
      followdirection = follow_right;
      mission.state = ms_followline;
      break;

    case ms_fwd:
      if (fwd(dist, 0.4, mission.time))
        mission.state = ms_turn;
      break;

    case ms_turn:
      // mission.state = ms_end;
      if (turn(angle, 0.4, odo.theta, mission.time)) {
        n = n - 1;
        if (n == 0)
          mission.state = ms_end;
        else
          mission.state = ms_fwd;
      }
      break;

    case ms_followline:
      if (control(dist, angle, 0.2, followdirection, mission.time))
        mission.state = ms_end;
      break;

    case ms_end:
      mot.cmd = mot_stop;
      // printf("%f %f %f %f %f %f %f %f %f %f\n", laserpar[0], laserpar[1],
      //        laserpar[2], laserpar[3], laserpar[4], laserpar[5], laserpar[6],
      //        laserpar[7], laserpar[8], laserpar[9]);
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
    case mot_move:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_move;
      break;

    case mot_turn:
      if (p->angle > 0)
        p->startpos = p->right_pos;
      else
        p->startpos = p->left_pos;
      p->curcmd = mot_turn;
      break;

    case ms_followline:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_control;

      p->cmd = 0;
      break;
    }
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

  double breaking_distance_current =
      (p->currentspeed * p->currentspeed) / (2 * max_accel);

  // printf("Theta %f, current angle %f, starting angle %f, target angle %f
  // \n",
  //        odo->theta, current_angle, p->startangle, p->angle);
  // printf("Speed is now %f, acc %f, speedcmd %f\n", p->currentspeed,
  // current_accel, p->speedcmd); printf("Angles are %f %f %f\n", p->angle,
  // odo->theta, p->startangle);

  double remaining_angle = p->finalangle - odo->theta;
  remaining_angle += (remaining_angle > M_PI)    ? -(2 * M_PI)
                     : (remaining_angle < -M_PI) ? (2 * M_PI)
                                                 : 0;
  double delta_velocity;

  double *normalised_sensor_data = linear_transformation(linesensor->data);

  int minLineSensorIndex =
      find_lowest_line_idx(normalised_sensor_data, p->followdirection);

  // printf("linesensor %d %d %d %d %d %d %d %d; idx=%d\n", linesensor->data[0],
  //        linesensor->data[1], linesensor->data[2], linesensor->data[3],
  //        linesensor->data[4], linesensor->data[5], linesensor->data[6],
  //        linesensor->data[7], minLineSensorIndex);

  // printf("%f %f %f %f %f %f %f %f %f %f\n", laserpar[0],
  //        laserpar[1], laserpar[2], laserpar[3],
  //        laserpar[4], laserpar[5], laserpar[6],
  //        laserpar[7], laserpar[8], laserpar[9]);

  // double center_of_mass = calculate_center_of_mass(normalised_sensor_data,
  // 1);

  // printf("CoM: %f\n", center_of_mass);

  switch (p->curcmd) {
  case mot_stop:
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
    break;
  case mot_move:
    if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    } else if ((p->right_pos + p->left_pos) / 2 - p->startpos >
               p->dist - breaking_distance_current) {
      p->currentspeed = -(max_accel * delta_time) + p->currentspeed;
      p->motorspeed_l = p->currentspeed;
      p->motorspeed_r = p->currentspeed;
    } else {
      p->currentspeed = (current_accel * delta_time) + p->currentspeed;

      p->motorspeed_l = p->currentspeed;
      p->motorspeed_r = p->currentspeed;
      // p->motorspeed_l = p->speedcmd;
      // p->motorspeed_r = p->speedcmd;
    }
    break;

  case mot_turn:
    if (p->angle > 0) {
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;

      if (current_angle >= p->angle) {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      } else if ((p->angle - current_angle) * (p->w / 2) <
                 breaking_distance_current / 2) {
        p->currentspeed = -(max_accel * delta_time) + p->currentspeed;
        p->motorspeed_r = p->currentspeed / 2;
        p->motorspeed_l = -p->currentspeed / 2;

      } else {
        p->currentspeed = (current_accel * delta_time) + p->currentspeed;
        p->motorspeed_r = p->currentspeed / 2;
        p->motorspeed_l = -p->currentspeed / 2;
      }
    } else {

      if (current_angle <= p->angle) {
        p->motorspeed_r = 0;
        p->motorspeed_l = 0;
        p->finished = 1;
      } else if ((p->angle - current_angle) * (p->w / 2) <
                 breaking_distance_current / 2) {
        p->currentspeed = -(max_accel * delta_time) + p->currentspeed;
        p->motorspeed_r = -p->currentspeed / 2;
        p->motorspeed_l = p->currentspeed / 2;

      } else {
        p->currentspeed = (current_accel * delta_time) + p->currentspeed;
        p->motorspeed_r = -p->currentspeed / 2;
        p->motorspeed_l = p->currentspeed / 2;
      }
    }

    break;

  case mot_control:
    // delta V and mot_turn/mot_move cases
    // K=3; delta_v = K * (final angle - current angle)

    // // delta_velocity = 10 * remaining_angle;
    delta_velocity = 1.25 * (normalised_sensor_data[MIDDLE_LINE_SENSOR] -
                             normalised_sensor_data[minLineSensorIndex]);
    // delta_velocity = .2 * center_of_mass;

    if (minLineSensorIndex == -1) {
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;
      p->finished = 1;
      break;
    }

    if (p->followdirection == follow_middle) {
      if (normalised_sensor_data[MIDDLE_LINE_SENSOR] < 0.1) {
        delta_velocity = 0;
      }
    }

    // if (center_of_mass != center_of_mass) {
    //   delta_velocity = 0;
    // }

    if (minLineSensorIndex < MIDDLE_LINE_SENSOR) {
      delta_velocity = -delta_velocity;
    }

    // printf("dv=%f;sensor_m=%f;sensor_min=%f\n", delta_velocity,
    // normalised_sensor_data[4], normalised_sensor_data[minLineSensorIndex]);

    // delta_velocity = 0.5 * center_of_mass;

    if ((p->right_pos + p->left_pos) / 2 - p->startpos > p->dist) {
      p->motorspeed_r = 0;
      p->motorspeed_l = 0;
      p->finished = 1;
    } else {
      p->motorspeed_r = (p->speedcmd + delta_velocity) / 2;
      p->motorspeed_l = (p->speedcmd - delta_velocity) / 2;
    }

    break;
  }
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
      } else if (i < MIDDLE_LINE_SENSOR) {
        foundRightLineIdx = i;
      } else if (i > MIDDLE_LINE_SENSOR) {
        foundLeftLineIdx = i;
      }
    }
  }

  // Stop at cross
  bool crossLine = true;
  for (int i = 1; i < 7; i++) {
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

int fwd(double dist, double speed, int time) {
  if (time == 0) {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.currentspeed = 0;
    mot.dist = dist;
    return 0;
  } else
    return mot.finished;
}

int turn(double angle, double speed, double startangle, int time) {
  if (time == 0) {

    mot.cmd = mot_turn;
    mot.speedcmd = speed;
    mot.currentspeed = 0;

    mot.startangle = startangle;

    mot.angle = angle;
    return 0;
  } else
    return mot.finished;
}

// ex7
int control(double dist, double finalangle, double speed, int followdirection,
            int time)

{
  if (time == 0) {
    mot.cmd = mot_control;
    mot.speedcmd = speed;
    mot.dist = dist;

    mot.currentspeed = 0;
    mot.finalangle = finalangle;
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
