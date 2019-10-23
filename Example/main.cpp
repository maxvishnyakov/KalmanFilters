#include <iostream>
#include "ConstVelocityModel.h"
#include "KalmanFilters/ExtendedKalmanFilter.h"

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
    KalmanFilters::ExtendedKalmanFilter<ConstVelocityModel> filter;
    cout << filter.validate_measurements(model, meas, 6) << endl;
    filter.update(model, meas, 6);
    model.state.print("state");
    return 0;
}
