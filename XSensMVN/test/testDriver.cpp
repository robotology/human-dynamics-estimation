/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "XSensLogger.h"
#include "XSensMVNDriver.h"

#include "xme.h"

#include <chrono>
#include <thread>

int main(int argc, const char* argv[])
{

    xsensmvn::DriverDataStreamConfig dsconf = {true, true, false};

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
        bd,
        dsconf};

    xsensmvn::XSensMVNDriver driver(conf);

    driver.configureAndConnect();

    xsInfo << std::endl << std::endl << "Connected, ready to be calibrated";

    if (driver.calibrate("Npose")) {
        xsInfo << std::endl << std::endl << "Calibrated, ready to acquire data";
    };

    if (driver.startAcquisition()) {
        xsInfo << std::endl << std::endl << "Recording";
        for (int j = 0; j < 50; ++j) {
            xsensmvn::LinkDataVector linkSample;
            linkSample = driver.getLinkDataSample();
            if (!linkSample.data.empty()) {
                double relTime = linkSample.time.relative;
                xsInfo << "time: " << relTime;
                xsInfo << linkSample.data.at(0).name
                       << " position:" << linkSample.data.at(1).position.at(0);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    xsInfo << "Done. Closing.";
    xsInfo << "Closing success: " << driver.terminate();
    return EXIT_SUCCESS;
}
