#include <iostream>
#include "ConstCarModel_A_R.h"
#include <string>
#include <tuple>
#include <vector>
#include <fstream>
#include "ExtendedKalmanFilter.h"

using namespace std;
/////////////////////////////////////////////////////////////
std::tuple<float, float> get_x_y_value(float a_meas, float r_meas)
{
    float x_meas_value = r_meas * sin(a_meas);
    float y_meas_value = r_meas * cos(a_meas);

    return std::make_tuple(x_meas_value, y_meas_value);
}

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
        std::string::size_type pos = temp.find("{");
        string a_temp = temp.substr(pos + 1);
        pos = a_temp.find(", ");
        string a_value = a_temp.substr(0 , pos);
        a_meas.push_back(stof(a_value));

        pos = a_temp.find("}");
        string r_temp = a_temp.substr(0, pos);
        pos = r_temp.find(", ");
        string r_value = r_temp.substr(pos + 1);
        r_meas.push_back(stof(r_value));
    }
    return std::make_tuple(a_meas, r_meas);
}

std::tuple<std::vector<float>, std::vector<float>> filter_values_x_y(std::vector<float> & a_meas, std::vector<float> & r_meas)
{
    ConstCarModel_A_R model;
    float x_start, y_start;
    std::tie(x_start, y_start) = get_x_y_value(a_meas[0], r_meas[0]);
    KalmanFilters::ExtendedKalmanFilter<ConstCarModel_A_R> filter;
    model.state = {x_start, y_start, 400, 400};
    model.P = {
        { 50,     0,      0,    0},
        {   0,   50,      0,    0},
        {   0,      0,   10,    0},
        {   0,      0,    0,    10}
    };
    ConstCarModel_A_R::measurement_vec meas;
    std::vector<float> x_output, y_output;
    for(int i = 0; i < 199; ++i)
    {
        meas = {a_meas[i], r_meas[i]};
        filter.update(model, meas, 1);
        x_output.push_back(model.state.at(0));
        y_output.push_back(model.state.at(1));
    }
    return std::make_tuple(x_output, y_output);
}

void write_out_values_x_y(std::vector<float> & x_output, std::vector<float> & y_output, ofstream & fout)
{
    for(int i = 0; i < 199; ++i)
    {
        fout<<"{"<<to_string(x_output[i])<<","
           <<to_string(y_output[i])<<"}"<<"\n";
    }
     fout<<"\n";
}


int main()
{
    ifstream fin;
    fin.open("/home/vishnyakov/work/python/filter/generate_a_r", ios_base::in);

    ofstream fout;
    fout.open("/home/vishnyakov/work/python/filter/filter_values_x_y_a_r", ios_base::out);

    //TODO: изменить на проверку: конец ли файла ?/////
    for(int i = 0; i < 5; ++i)
    {
        std::vector<float> a_meas, r_meas;
        std::tie(a_meas, r_meas) = get_measurements_a_r(fin);
        std::vector<float> x_output, y_output;
        std::tie(x_output, y_output) = filter_values_x_y(a_meas, r_meas);
        write_out_values_x_y(x_output, y_output, fout);
    }

    fin.close();
    fout.close();

    return 0;
}




