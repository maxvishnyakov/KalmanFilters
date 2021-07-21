#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "ConstCarModel_X.h"
#include "KalmanFilter.h"

using namespace std;

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
    ConstCarModel_X model;
    KalmanFilters::KalmanFilter<ConstCarModel_X> filter;
    model.state = {input[0], 0};
    ConstCarModel_X::measurement_vec meas;
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
    fin.open("/home/vishnyakov/work/python/filter/meas_values_test_second", ios_base::in);
    ofstream fout;
    fout.open( "/home/vishnyakov/work/python/filter/filter_values_second", ios_base::out);

    for(int i = 0; i <= 1000; ++i)
    {
        fout<<"x"<< i<<",";
    }
    fout<<"\n";

    for(int i = 0; i <3; ++i)
    {
        auto input = get_measurements(fin);
        auto output = filter_values(input);
        write_out_values(output, fout);
    }

    fout.close();
    fin.close();

    return 0;
}
