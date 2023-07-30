// Copyright 2023 Ar-Ray-code.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "feetech_scs_interface/feetech_scs_utils.hpp"
#include <iostream>

namespace feetech_scs_interface
{

int SCS0009::data2angle(int data)
{
  return (int)(data * 360 / SCS0009_ANGLE_360);
}

int SCS0009::angle2data(const int angle)
{
  return angle * SCS0009_ANGLE_360 / 360;
}

}  // namespace feetech_scs_interface
