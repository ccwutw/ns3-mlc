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

#ifndef GCL_GYM_ENTITY_H
#define GCL_GYM_ENTITY_H

#include <ns3/opengym-module.h>
#include <ns3/nstime.h>

namespace ns3 {

class GclGymEnv : public OpenGymEnv
{
public:
  GclGymEnv ();
  GclGymEnv (Time stepTime);
  virtual ~GclGymEnv ();
  static TypeId GetTypeId (void);
  virtual void DoDispose ();

  void SetReward(float value);
  void SetPenalty(float value);
  
  // OpenGym interface
  Ptr<OpenGymSpace> GetActionSpace();
  Ptr<OpenGymSpace> GetObservationSpace();
  Ptr<OpenGymDataContainer> GetObservation();
  bool GetGameOver();
  float GetReward();
  std::string GetExtraInfo();
  bool ExecuteActions(Ptr<OpenGymDataContainer> action);

  // trace packets to calaulate latency and packet loss
  
  // mobility management interface

  // functions used to collect observations

private:
  void ScheduleNextStateRead();

  Time m_interval;
  bool isGameOver;
  
  // reward
  float m_reward;
  float m_penalty;
};

}


#endif // GCL_GYM_ENTITY_H
