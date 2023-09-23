// Copyright 2023 fateshelled.
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

#pragma once

#define STS3032_ANGLE_360 4096

namespace feetech_sts_interface
{

namespace STS3032
{
float data2angle(int);
int angle2data(const float);
}

}  // namespace feetech_sts_interface
