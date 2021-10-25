#include <iostream>
#include <tuple>
#include <vector>
#include <string>
#include "../ConstCarModel_X_Y.h"
#include <fstream>
#include "KalmanFilter.h"

using namespace std;

std::tuple<std::vector<float>, std::vector<float>> get_measurements_x_y(ifstream & fin)
{
    std::vector<float> x_meas;
    std::vector<float> y_meas;
    string temp;

    while(true)
    {
        getline(fin, temp);
        if(!temp.size())
        {
            break;
        }
        std::string::size_type pos = temp.find("{");
        string x_temp = temp.substr(pos + 1);
        pos = x_temp.find(", ");
        string x_value = x_temp.substr(0 , pos);
        x_meas.push_back(stof(x_value));

        pos = x_temp.find("}");
        string y_temp = x_temp.substr(0, pos);
        pos = y_temp.find(", ");
        string y_value = y_temp.substr(pos + 1);
        y_meas.push_back(stof(y_value));
    }
    return std::make_tuple(x_meas, y_meas);
}

std::tuple<std::vector<float>, std::vector<float>> filter_values_x_y(std::vector<float> & x_meas, std::vector<float> & y_meas)
{
    ConstCarModel_X_Y model;
    KalmanFilters::KalmanFilter<ConstCarModel_X_Y> filter;
    model.state = {x_meas[0], y_meas[0], 0, 0};
    ConstCarModel_X_Y::measurement_vec meas;
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

void write_out_values_a_y(std::vector<float> & x_output, std::vector<float> & y_output, ofstream & fout)
{
    for(int i = 0; i < 200; ++i)
    {
        fout<<to_string(x_output[i])<<" "
           <<to_string(y_output[i])<<"\n";
    }
    fout<<"\n";
}


int main()
{
    ifstream fin;
    fin.open("/home/vishnyakov/work/python/filter/meas_values_x_y", ios_base::in);

    ofstream fout;
    fout.open("/home/vishnyakov/work/python/filter/filter_values_x_y", ios_base::out);

    for(int i = 0; i < 100; ++i)
    {
        std::vector<float> x_meas, y_meas;
        std::tie(x_meas, y_meas) = get_measurements_x_y(fin);
        std::vector<float> x_output, y_output;
        std::tie(x_output, y_output) = filter_values_x_y(x_meas, y_meas);
        write_out_values_a_y(x_output, y_output, fout);
    }

    fin.close();
    fout.close();

    return 0;
}

























    //std::vector<float> output_x;
    //std::vector<float> output_Y;


    /*
    ifstream fin_x;
    fin_x.open("/home/vishnyakov/work/python/filter/meas_values_x", ios_base::in);

    ifstream fin_y;
    fin_y.open("/home/vishnyakov/work/python/filter/meas_values_y", ios_base::in);

    ofstream fout_x;
    fout_x.open( "/home/vishnyakov/work/python/filter/filter_values_x", ios_base::out);

    ofstream fout_y;
    fout_y.open( "/home/vishnyakov/work/python/filter/filter_values_y", ios_base::out);


    std::vector<float> input_x;
    string temp;
    while(true)
    {
        getline(fin_x, temp);
        if(!temp.size())
        {
            break;
        }
        input_x.push_back(stof(temp));
    }

    std::vector<float> input_y;
    string tempy;
    while(true)
    {
        getline(fin_y, tempy);
        if(!tempy.size())
        {
            break;
        }
        input_y.push_back(stof(tempy));
    }
    for(float n : input)
    {
        meas = n;
        filter.update(model, meas, 1);
        output.push_back(model.state.at(0));
    }
    */



    /*
    for(int i = 0; i <= 1000; ++i)
    {
        fout<<"x"<< i<<",";
    }
    fout<<"\n";
    */


/*
std::vector<float> get_measurements(ifstream & fin)
{
    std::vector<float> input;
    string temp;
    while(true)
    {
        getline(fin, temp);
        if(!temp.size())
        {
            break;
        }
        input.push_back(stof(temp));
    }
    return input;
}

std::vector<float> filter_values(const std::vector<float> & input)
{
    ConstCarModel model;
    KalmanFilters::KalmanFilter<ConstCarModel> filter;
    model.state = {input[0], 0};
    ConstCarModel::measurement_vec meas;
    std::vector<float> output;
    for(float n : input)
    {
        meas = n;
        filter.update(model, meas, 1);
        output.push_back(model.state.at(0));
    }
    return output;
}

void write_out_values(const std::vector<float> & output, ofstream & fout)
{
    for(float n : output)
    {
       fout<<to_string(n)<<",";
    }
    fout<<"\n";
}


int main()
{
    ifstream fin;
    fin.open("/home/vishnyakov/work/python/filter/meas_values_test", ios_base::in);
    ofstream fout;
    fout.open( "/home/vishnyakov/work/python/filter/filter_values_new", ios_base::out);

    for(int i = 0; i <= 1000; ++i)
    {
        fout<<"x"<< i<<",";
    }
    fout<<"\n";
    for(int i = 0; i <5000; ++i)
    {
        auto input = get_measurements(fin);
        auto output = filter_values(input);
        write_out_values(output, fout);
    }

    fout.close();
    fin.close();

    return 0;
*/

    /*
    ConstCarModel model;
    model.state = {0, 50};
    ConstCarModel::measurement_vec meas = {1.0f};
    KalmanFilters::KalmanFilter<ConstCarModel> filter;
    cout << filter.validate_measurements(model, meas, 6) << endl;
    filter.update(model, meas, 6);
    model.state.print("state");

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
    */
