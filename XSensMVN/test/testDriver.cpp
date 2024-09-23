// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "XSensLogger.h"
#include "XSensMVNDriver.h"

#include "xme.h"

#include <chrono>
#include <thread>

int main(int argc, const char* argv[])
{

    xsensmvn::DriverDataStreamConfig dsconf = {true, true, true};

    xsensmvn::bodyDimensions bd = {{"ankleHeight", 0.07},
                                   {"armSpan", 1.71},
                                   {"bodyHeight", 1.71},
                                   {"footSize", 0.26},
                                   {"hipHeight", 0.87},
                                   {"hipWidth", 0.25},
                                   {"kneeHeight", 0.50},
                                   {"shoulderWidth", 0.34},
                                   {"shoeSoleHeight", 0.02}};

    xsensmvn::DriverConfiguration conf = {
        "C:/Program Files/Xsens/MVN SDK 2018.0.3/SDK Files/rundeps",
        "FullBody",
        "",
        "Tpose",
        xsensmvn::CalibrationQuality::FAILED,
        20,
        120,
        bd,
        dsconf};

    xsInfo << std::endl << std::endl << "Creating";
    xsensmvn::XSensMVNDriver driver(conf);

    xsInfo << std::endl << std::endl << "Configuring";

    driver.configureAndConnect();

    xsInfo << std::endl << std::endl << "Connected, ready to be calibrated";

    if (driver.calibrate("Npose")) {
        xsInfo << std::endl << std::endl << "Calibrated, ready to acquire data";
    };

    if (driver.startAcquisition()) {
        xsInfo << std::endl << std::endl << "Recording";

        const auto jointLabels = driver.getSuitJointLabels();
        xsInfo << "----------- Joint Labels -------------";
        for (const auto& j : jointLabels) {
            xsInfo << " - " << j;
        }
        xsInfo << "--------------------------------------";

        const auto linkLabels = driver.getSuitLinkLabels();
        xsInfo << "----------- Link Labels -------------";
        for (const auto& l : linkLabels) {
            xsInfo << " - " << l;
        }
        xsInfo << "--------------------------------------";

        for (int j = 0; j < 50; ++j) {
            xsensmvn::LinkDataVector linkSample;
            linkSample = driver.getLinkDataSample();
            if (!linkSample.data.empty()) {
                //  xsInfo << "Xsens relative time: " << linkSample.time->relative;
                //  xsInfo << "Xsens absolute time: " << linkSample.time->absolute;
                xsInfo << std::endl
                       << std::endl
                       << "System time (UNIX time): " << linkSample.time->systemTime;

                for (size_t l = 0; l < linkLabels.size(); ++l) {
                    xsInfo << linkSample.data.at(l).name
                           << " position: " << linkSample.data.at(l).position.at(0) << " "
                           << linkSample.data.at(l).position.at(1) << " "
                           << linkSample.data.at(l).position.at(2);
                }
            }

            xsensmvn::JointDataVector jointSample = driver.getJointDataSample();
            if (!jointSample.data.empty()) {
                xsInfo << std::endl
                       << std::endl
                       << "System time (UNIX time): " << jointSample.time->systemTime;
                for (size_t j = 0; j < jointLabels.size(); ++j) {
                    xsInfo << jointSample.data.at(j).name
                           << " jointAngles XYZ: " << jointSample.data.at(j).angles.at(0) << " "
                           << jointSample.data.at(j).angles.at(1) << " "
                           << jointSample.data.at(j).angles.at(2);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    xsInfo << "Done. Closing.";
    xsInfo << "Closing success: " << driver.terminate();
    return EXIT_SUCCESS;
}
