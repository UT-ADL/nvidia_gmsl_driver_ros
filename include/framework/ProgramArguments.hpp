/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2014-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_COMMON_PROGRAMARGUMENTS_HPP_
#define SAMPLES_COMMON_PROGRAMARGUMENTS_HPP_

#include <map>
#include <string>
#include <vector>

// EXAMPLE USECASE:
//
//    ProgramArguments arguments(
//    { ProgramArguments::Option_t("optional_option", "with default value"),
//    ProgramArguments::Option_t("required_option")
//    }
//    );
//
//    if (!arguments.parse(argc, argv)) exit(0); // Exit if not all require arguments are provided
//    printf("Program Arguments:\n%s\n", arguments.printList());
//
class ProgramArguments
{
  public:
    struct Option_t {
        Option_t(const char *option_, const char *default_value_, const char *help_)
        {
            option        = option_;
            default_value = default_value_;
            help          = help_;
            required      = false;
            parsed        = false;
            value         = default_value;
        }
        Option_t(const char *option_, std::nullptr_t, const char *help_)
        {
            option        = option_;
            default_value = "";
            help          = help_;
            required      = true;
            parsed        = false;
            value         = "";
        }

        Option_t(const char *option_, const char *default_value_)
        {
            option        = option_;
            default_value = default_value_;
            required      = false;
            parsed        = false;
            value         = default_value;
        }

        Option_t(const char *option_)
        {
            option        = option_;
            default_value = "";
            required      = true;
            parsed        = false;
        }

        std::string option;
        std::string default_value;
        std::string help;
        bool required;

        bool parsed;
        std::string value;
    };

  public:
    ProgramArguments() {}
    ProgramArguments(int argc, const char **argv, const std::vector<Option_t>& options, const char* description = nullptr);
    ProgramArguments(const std::vector<Option_t>& options);
    ~ProgramArguments();

    bool parse(const int argc, const char **argv);

    bool has(const char *name) const;
    bool enabled(const char *name) const;
    void addOption(const Option_t &newOption);

    const std::string &get(const char *name) const;
    void set(const char* option, const char* value);

    // Will be shown before the help text
    void setDescription(const char* description);

    /// Displays a message with info about the possible parameters
    void printHelp() const;

    /// Returns a string with info about the parameter values
    std::string printList() const;

    /// Returns all the parsed parameters as a single string
    std::string parameterString() const;

  private:
    static std::string m_empty;

    std::string m_description = {};
    std::map<std::string, Option_t> arguments;
};

#endif // SAMPLES_COMMON_PROGRAMARGUMENTS_HPP_
