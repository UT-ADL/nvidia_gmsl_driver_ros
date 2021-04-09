/*
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  All rights reserved.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
  Modified from Nvidia SDK - Camera gmsl
  Author: Punnu Phairatt
  Initial Date: 10/05/18
*/

#include "DeviceArguments.hpp"

namespace DriveWorks {

DeviceArguments::DeviceArguments(const VecPairStrStr &options) {
  for (const PairStrStr &option: options) {
    arguments.insert(option);
  }
}

void DeviceArguments::printArguments() {
  for (auto arg: arguments) {
    std::cout << arg.first << "    " << arg.second << std::endl;
  }
}

const std::string &DeviceArguments::get(const std::string &name) const {
  auto it = arguments.find(name);
  if (it == arguments.end()) {
    std::cout << "Get error: Missing device argument: " << name << std::endl;
    return empty_string;
  } else {
    return it->second;
  }
}

bool DeviceArguments::set(const std::string &name, const std::string &new_value) {
  auto it = arguments.find(name);
  if (it == arguments.end()) {
    std::cout << "Set error: Missing argument: " << name << std::endl;
    return false;
  } else {
    arguments[name] = new_value;
    return true;
  }
}

}
