/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 Aalborg University
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Chien-Cheng Wu <ccw@es.aau.dk>
 */

#include "gcl-drl-env.h"
#include <ns3/object.h>
#include <ns3/core-module.h>
#include <ns3/node-list.h>
#include <ns3/log.h>
#include <ns3/network-module.h>
#include <ns3/mobility-module.h>
#include <ns3/internet-module.h>
#include <ns3/lte-module.h>

#include <sstream>
#include <iostream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("GclGymEnv");

NS_OBJECT_ENSURE_REGISTERED (GclGymEnv);

GclGymEnv::GclGymEnv ()
{
  NS_LOG_FUNCTION (this);
  m_interval = Seconds(0.1);

  Simulator::Schedule (Seconds(0.0), &GclGymEnv::ScheduleNextStateRead, this);
}

GclGymEnv::GclGymEnv (Time stepTime)
{
  NS_LOG_FUNCTION (this);
  m_interval = stepTime;

  Simulator::Schedule (Seconds(0.0), &GclGymEnv::ScheduleNextStateRead, this);
}

GclGymEnv::~GclGymEnv ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
GclGymEnv::GetTypeId (void)
{
  static TypeId tid = TypeId ("GclGymEnv")
    .SetParent<OpenGymEnv> ()
    .SetGroupName ("OpenGym")
    .AddConstructor<GclGymEnv> ()
  ;
  return tid;
}

void
GclGymEnv::DoDispose ()
{
  NS_LOG_FUNCTION (this);
}

void
GclGymEnv::SetReward(float value)
{
  NS_LOG_FUNCTION (this);
  m_reward = value;
}

void
GclGymEnv::SetPenalty(float value)
{
  NS_LOG_FUNCTION (this);
  m_penalty = value;
}

void
GclGymEnv::ScheduleNextStateRead ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Schedule (m_interval, &GclGymEnv::ScheduleNextStateRead, this);
  Notify();
}

/*
Define observation space
*/
Ptr<OpenGymSpace>
GclGymEnv::GetObservationSpace()
{
  //latency
  //packet loss
  //velocity
  //rss
  //neighbor cell list
  uint32_t parameterNum = 5;
  float low = 0.0;
  float high = 1000000000.0;
  std::vector<uint32_t> shape = {parameterNum,};
  std::string dtype = TypeNameGet<uint32_t> ();

  Ptr<OpenGymDiscreteSpace> discrete = CreateObject<OpenGymDiscreteSpace> (parameterNum);
  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);

  Ptr<OpenGymDictSpace> space = CreateObject<OpenGymDictSpace> ();
  space->Add("myVector", box);
  space->Add("myValue", discrete);

  NS_LOG_UNCOND ("MyGetObservationSpace: " << space);
  return space;
}

/*
Define action space
*/
Ptr<OpenGymSpace>
GclGymEnv::GetActionSpace()
{
  //latency
  //packet loss
  //velocity
  //rss
  //neighbor cell list

  uint32_t parameterNum = 5;
  float low = 0.0;
  float high = 1000000000.0;
  std::vector<uint32_t> shape = {parameterNum,};
  std::string dtype = TypeNameGet<uint32_t> ();

  //generate new neighbor cell list
  Ptr<OpenGymDiscreteSpace> discrete = CreateObject<OpenGymDiscreteSpace> (parameterNum);
  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);

  Ptr<OpenGymDictSpace> space = CreateObject<OpenGymDictSpace> ();
  space->Add("myActionVector", box);
  space->Add("myActionValue", discrete);

  NS_LOG_UNCOND ("MyGetActionSpace: " << space);
  return space;
}

/*
Define game over condition
*/
bool
GclGymEnv::GetGameOver()
{
  isGameOver = false;
  bool test = false;
  static float stepCounter = 0.0;
  stepCounter += 1;
  if (stepCounter == 10 && test) {
      isGameOver = true;
  }
  NS_LOG_UNCOND ("MyGetGameOver: " << isGameOver);
  return isGameOver;
}

/*
Collect observations
*/
Ptr<OpenGymDataContainer>
GclGymEnv::GetObservation()
{
  uint32_t parameterNum = 5;
  uint32_t low = 0.0;
  uint32_t high = 1000000000.0;
  Ptr<UniformRandomVariable> rngInt = CreateObject<UniformRandomVariable> ();

  std::vector<uint32_t> shape = {parameterNum,};
  Ptr<OpenGymBoxContainer<uint32_t> > box = CreateObject<OpenGymBoxContainer<uint32_t> >(shape);

  // generate random data
  for (uint32_t i = 0; i<parameterNum; i++){
    uint32_t value = rngInt->GetInteger(low, high);
    box->AddValue(value);
  }

  Ptr<OpenGymDiscreteContainer> discrete = CreateObject<OpenGymDiscreteContainer>(parameterNum);
  uint32_t value = rngInt->GetInteger(low, high);
  discrete->SetValue(value);

  Ptr<OpenGymDictContainer> data = CreateObject<OpenGymDictContainer> ();
  data->Add("myVector",box);
  data->Add("myValue",discrete);

  // Print data from tuple
  Ptr<OpenGymBoxContainer<uint32_t> > mbox = DynamicCast<OpenGymBoxContainer<uint32_t> >(data->Get("myVector"));
  Ptr<OpenGymDiscreteContainer> mdiscrete = DynamicCast<OpenGymDiscreteContainer>(data->Get("myValue"));
  NS_LOG_UNCOND ("MyGetObservation: " << data);
  NS_LOG_UNCOND ("---" << mbox);
  NS_LOG_UNCOND ("---" << mdiscrete);

  return data;
}

/*
Define reward function
*/
float
GclGymEnv::GetReward()
{
  NS_LOG_INFO("MyGetReward: " << m_reward);
  return m_reward;
}

/*
Define extra info. Optional
*/
std::string
GclGymEnv::GetExtraInfo()
{
  std::string myInfo = "testInfo";
  //do something
  NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);
  return myInfo;
}

/*
Execute received actions
*/
bool
GclGymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
  Ptr<OpenGymDictContainer> dict = DynamicCast<OpenGymDictContainer>(action);
  Ptr<OpenGymBoxContainer<uint32_t> > box = DynamicCast<OpenGymBoxContainer<uint32_t> >(dict->Get("myActionVector"));
  Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(dict->Get("myActionValue"));

  NS_LOG_UNCOND ("MyExecuteActions: " << action);
  NS_LOG_UNCOND ("---" << box);
  NS_LOG_UNCOND ("---" << discrete);
  return true;
}

} // ns3 namespace
