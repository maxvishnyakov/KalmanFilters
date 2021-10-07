#include <iostream>
#include "ConstCarMode_A_R.h"
#include <string>
#include <tuple>
#include <vector>
#include <fstream>
#include "ExtendedKalmanFilter.h"

using namespace std;

/*
std::tuple<float, float> get_x_y(float a_meas, float r_meas)
{
    float x_meas = r_meas * sin(a_meas * M_PI / 180);
    float y_meas = r_meas * cos(a_meas * M_PI / 180);

    return std::make_tuple(x_meas, y_meas);
}
*/


std::tuple<std::vector<float>, std::vector<float>> get_measurements_a_r(ifstream & fin)
{
    std::vector<float> a_meas;
    std::vector<float> r_meas;
    string temp;

    while(true)
    {
        getline(fin, temp);
        if(!temp.size())
        {
            break;
        }
        std::string::size_type pos = temp.find("[");
        string x_temp = temp.substr(pos + 1);
        pos = x_temp.find(", ");
        string x_value = x_temp.substr(0 , pos);
        a_meas.push_back(stof(x_value));

        pos = x_temp.find("]");
        string y_temp = x_temp.substr(0, pos);
        pos = y_temp.find(", ");
        string y_value = y_temp.substr(pos + 1);
        r_meas.push_back(stof(y_value));
    }
    return std::make_tuple(a_meas, r_meas);
}

std::tuple<std::vector<float>, std::vector<float>> filter_values_a_r(std::vector<float> & x_meas, std::vector<float> & y_meas)
{
    ConstCarModel_A_R model;
    KalmanFilters::ExtendedKalmanFilter<ConstCarModel_A_R> filter;
    model.state = {x_meas[0], y_meas[0], 0, 0};
    ConstCarModel_A_R::measurement_vec meas;
    std::vector<float> x_output, y_output;
    for(int i = 0; i < 200; ++i)
    {
        meas = {x_meas[i], y_meas[i]};
        filter.update(model, meas, 1);
        x_output.push_back(model.state.at(0));
        y_output.push_back(model.state.at(1));
    }
    return std::make_tuple(x_output, y_output);
}

void write_out_values_a_r(std::vector<float> & a_output, std::vector<float> & r_output, ofstream & fout)
{
    for(int i = 0; i < 200; ++i)
    {
        fout<<to_string(a_output[i])<<" "
           <<to_string(r_output[i])<<"\n";
    }
    fout<<"\n";
}


int main()
{
    ifstream fin;
    fin.open("/home/vishnyakov/work/python/filter/meas_values_x_y_a_r", ios_base::in);

    ofstream fout;
    fout.open("/home/vishnyakov/work/python/filter/filter_values_x_y_a_r", ios_base::out);

    for(int i = 0; i < 100; ++i)
    {
        std::vector<float> a_meas, r_meas;
        std::tie(a_meas, r_meas) = get_measurements_a_r(fin);
        std::vector<float> a_output, r_output;
        std::tie(a_output, r_output) = filter_values_a_r(a_meas, r_meas);
        write_out_values_a_r(a_output, r_output, fout);
    }

    fin.close();
    fout.close();

    return 0;
}




