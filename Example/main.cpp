#include <iostream>
#include "ConstVelocityModel.h"
#include "ExtendedKalmanFilter.h"

using namespace std;

int main()
{
    ConstVelocityModel::stations_pos st_pos = {{0, 0}, {8000, 0}};
    ConstVelocityModel model(st_pos);
    model.state = {10, 100, 10, 0};
    model.P = {
        {1000000,    0,   0,    0},
        {   0, 1000000,   0,    0},
        {   0,    0, 5000000,    0},
        {   0,    0,   0,  5000000}
    };
    ConstVelocityModel::measurement_vec meas = {1.0f, 1.1f};
    ExtendedKalmanFilter<ConstVelocityModel> filter(model);
    cout << filter.validate_measurements(meas, 6) << endl;
    filter.update(meas, 6);
    model.state.print("state");
    return 0;
}
