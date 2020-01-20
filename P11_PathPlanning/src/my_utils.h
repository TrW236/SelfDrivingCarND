#include<vector>
#include<cmath>
#include "json.hpp"
#include <iostream>

using namespace std;


namespace my_utils {
    class CoordinatesConverter {
    public:
        double car_x;
        double car_y;
        double car_yaw;

        CoordinatesConverter(double car_x, double car_y, double car_yaw) {  // yaw must be rad
            this->car_x = car_x;
            this->car_y = car_y;
            this->car_yaw = car_yaw;
        }

        ~CoordinatesConverter() {}

        vector<double> world2car(vector<double> worldXY) {
            vector<double> carXY(2);
            carXY[0] = (cos(-car_yaw) * (worldXY[0] - car_x) - sin(-car_yaw) * (worldXY[1] - car_y));
            carXY[1] = (sin(-car_yaw) * (worldXY[0] - car_x) + cos(-car_yaw) * (worldXY[1] - car_y));
            return carXY;
        }

        vector<double> car2world(vector<double> carXY) {
            vector<double> worldXY(2);
            worldXY[0] = (cos(car_yaw) * carXY[0] - sin(car_yaw) * carXY[1] + car_x);
            worldXY[1] = (sin(car_yaw) * carXY[0] + cos(car_yaw) * carXY[1] + car_y);
            return worldXY;
        }
    };
}

namespace states_machine{
    int const MAX_VEL = 1, FOLLOW_CAR = 2, PREP_CHANGE = 3, CHANGING = 4;

    double follow_vel;
    double v_max= 50 * 0.98;
    int idx_too_close = -1;
    double danger_dist = 30;
    double up_tol_check_lr = 50;
    double low_tol_check_lr = 10;

    double calc_v(int idx, vector<vector<double>> sensor_fusion){
        double vx = sensor_fusion[idx][3];
        double vy = sensor_fusion[idx][4];
        return sqrt(vx * vx + vy * vy); // m/s
    }

    bool check_left(double *lane, double car_s, vector<vector<double>> sensor_fusion){
        if (*lane == 0) return false;
        else {
            for (int i = 0; i < sensor_fusion.size(); ++i) {
                double d = sensor_fusion[i][6];
                if (d < 4.0 * (*lane) && d > 4.0 * (*lane) - 4.0) {
                    double v = calc_v(i, sensor_fusion);
                    double s = sensor_fusion[i][5];
                    double check_s = s + 1.25 * v;
                    if (check_s > car_s + low_tol_check_lr && check_s < car_s + up_tol_check_lr) return false;
                }
            }
            return true;
        }
    }

    bool check_right(double *lane, double car_s, vector<vector<double>> sensor_fusion){
        if (*lane == 2) return false;
        else {
            for (int i = 0; i < sensor_fusion.size(); ++i) {
                double d = sensor_fusion[i][6];
                if (d < 8.0 + 4.0 * (*lane) && d > 4.0 + 4.0 * (*lane)) {
                    double v = calc_v(i, sensor_fusion);
                    double s = sensor_fusion[i][5];
                    double check_s = s + 1.25 * v;
                    if (check_s > car_s + low_tol_check_lr && check_s < car_s + up_tol_check_lr) return false;
                }
            }
            return true;
        }
    }

    void follow_car(int *state, double *lane, double *vel_ref, double car_s, vector<vector<double>> sensor_fusion) {
        if (idx_too_close == -1) follow_vel = v_max;
        else {
            double d_follow = sensor_fusion[idx_too_close][6];
            follow_vel = calc_v(idx_too_close, sensor_fusion) / 0.44704;
            double follow_s = sensor_fusion[idx_too_close][5];

            if (follow_vel > v_max || (d_follow > 4.0 + (*lane)*4.0 || d_follow < (*lane) * 4.0)) {
                *state = MAX_VEL;
                follow_vel = v_max;
                idx_too_close = -1;
                return;
            }

            if (follow_s - car_s < danger_dist){
                follow_vel -= 1.0;
            }

            if (*state==FOLLOW_CAR && follow_vel < 45.0){
                *state = PREP_CHANGE;
                return;
            }
        }

        double vel_dec = 0.5;
        double vel_inc = 0.25;
        if (*vel_ref > follow_vel + vel_dec) {
            *vel_ref -= vel_dec;
        } else if (*vel_ref < follow_vel - vel_inc) {
            *vel_ref += vel_inc;
        } else {
            *vel_ref = follow_vel;
        }
    }

    void prep_change(int *state, double *lane, double *vel_ref, double car_s, vector<vector<double>> sensor_fusion){
        follow_car(state, lane, vel_ref, car_s, sensor_fusion);
        bool left = check_left(lane, car_s, sensor_fusion);
        if (left) {
            (*lane)--;
            *state = CHANGING;
            idx_too_close = -1;
            follow_vel = *vel_ref;
            return;
        }
        bool right = check_right(lane, car_s, sensor_fusion);
        if (right) {
            (*lane)++;
            *state = CHANGING;
            idx_too_close = -1;
            follow_vel = *vel_ref;
            return;
        }
//        cout << "check left: "<< left << endl;
//        cout << "check right: " << right << endl;

    }

    void change_lane(int *state, double *lane, double *vel_ref, double car_s, double car_d, vector<vector<double>> sensor_fusion) {
        // follow_vel = v_max * 0.98;
        double vel_dec = 0.5;
        double vel_inc = 0.25;
        if (*vel_ref > follow_vel + vel_dec) {
            *vel_ref -= vel_dec;
        } else if (*vel_ref < follow_vel - vel_inc) {
            *vel_ref += vel_inc;
        } else {
            *vel_ref = follow_vel;
        }

        if (car_d<(*lane)*4.0 + 2.2 && car_d >(*lane)*4.0 + 1.8){
            *state = MAX_VEL;
        }

    }

    void check_close(bool *too_close, const double *lane, double car_s, vector<vector<double>> sensor_fusion) {

        for (int i = 0; i < sensor_fusion.size(); ++i) {
            double d = sensor_fusion[i][6];
            if (d < 5.0 + 4.0 * (*lane) && d > 4.0 * (*lane) - 1.0) {  // todo tolerance (at the moment is 1.0 meter)
                if (sensor_fusion[i][5] > car_s && (double) sensor_fusion[i][5] - car_s < danger_dist) {
                    *too_close = true;
                    idx_too_close = i;
                    follow_vel = calc_v(idx_too_close, sensor_fusion) / 0.44704;

                }
            }
        }
    }



    void max_vel(int *state, double *lane, double *vel_ref, double car_s, vector<vector<double>> sensor_fusion) {
//        double my_d = 2.0 + 4.0 * (*lane);

        bool too_close=false;

        check_close(&too_close, lane, car_s, sensor_fusion);
//        cout << "too_close: "<<too_close<<endl;
        if (too_close) {
            *state = FOLLOW_CAR;
            follow_car(state, lane, vel_ref, car_s, sensor_fusion);
        } else {
            follow_car(state, lane, vel_ref, car_s, sensor_fusion);
        }

    }



    void calc_state(int *state, double *lane, double *vel_ref, double car_s, double car_d, vector<vector<double>> sensor_fusion) {

        switch (*state) {
            case MAX_VEL:
                max_vel(state, lane, vel_ref, car_s, sensor_fusion);
                cout << "state: MAX VELOCITY" << endl;
                break;
            case FOLLOW_CAR:
                follow_car(state, lane, vel_ref, car_s, sensor_fusion);
                cout << "state: FOLLOW CAR" << endl;
                break;
            case PREP_CHANGE:
                prep_change(state, lane, vel_ref, car_s, sensor_fusion);
                cout << "state: PREPARE LANE CHANGING" << endl;
                break;
            case CHANGING:
                change_lane(state, lane, vel_ref, car_s, car_d, sensor_fusion);
                cout << "state: LANE CHANGING" << endl;
            default:
                break;
        }
    }



}