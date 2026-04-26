#pragma once

#include "bitbot_cifx/kernel/cifx_kernel.hpp"
#include "bitbot_cifx/device/joint_elmo_pushrod.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <fstream>
#include <sstream>

enum Events
{
  InitPos = 1001,
  ToFallPos1 = 1002,
  ToFallPos2 = 1003,
  ChangeMode = 1004,
};

enum class States : bitbot::StateId
{
  Waiting = 2001,
  InitPos = 2002,
  ToFallPos1 = 2003,
  ToFallPos2 = 2004,
  ChangeMode = 2005,
};

extern bitbot::JointElmoPushrod *joint_x;

struct UserData
{
};

using CifxKernel = bitbot::CifxKernel<UserData, "setup_time", "solve_time", "pin_time">;

void ConfigFunc(const bitbot::CifxBus &bus, UserData &);

std::optional<bitbot::StateId> EventInitPos(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventToFallPos1(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventToFallPos2(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventChangeMode(bitbot::EventValue value, UserData &user_data);

void StateWaiting(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateInitPos(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateToFallPos1(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateToFallPos2(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateChangeMode(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);

static std::vector<std::vector<double>> ReadCSV(std::string filename)
{
  using namespace std;
  vector<vector<double>> data;
  ifstream file(filename);

  string line;
  while (getline(file, line))
  {
    stringstream liness(line);
    string cell;
    vector<double> row;

    while (getline(liness, cell, ','))
    {
      row.push_back(stod(cell));
    }

    if (!row.empty())
      data.push_back(row);
  }

  return data;
}
