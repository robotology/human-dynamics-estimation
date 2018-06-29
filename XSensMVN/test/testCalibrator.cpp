/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "XSensCalibrationQualities.h"
#include "XSensMVNCalibrator.h"

#include "xme.h"

int main(int argc, const char* argv[])
{
    // Create a licence object
    XmeLicense lic;

    // Hardcoded path to the runtime dependencies
    std::string rundepsFolder = "C:/Program Files/Xsens/MVN SDK 2018.0.3/SDK Files/rundeps";

    // Configure the paths to the runtime required folders
    xmeSetPaths(rundepsFolder.c_str(), "", "", true);

    try {
        XmeControl* ctl = XmeControl::construct();

        // Connect to the suit
        xsInfo << "Starting scan";
        ctl->setScanMode(true);

        while (!ctl->status().isConnected())
            std::this_thread::sleep_for(std::chrono::microseconds(10));

        ctl->setScanMode(false);

        xsInfo << "Suit connected.";

        // List the available calibration types
        XsStringArray calibTypes = ctl->calibrationLabelList();
        xsInfo << "Calibration types:";
        for (auto& type : calibTypes) {
            xsInfo << type;
        }

        // Create the calibrator
        xsensmvn::XSensMVNCalibrator calib(*ctl, xsensmvn::CalibrationQuality::ACCEPTABLE);

        // Perform the calibration
        calib.calibrateWithType(std::string("Npose"));

        // Disconnect from the XSens suit and wait for a positive feedback
        ctl->disconnectHardware();
        while (ctl->status().isConnected())
            std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    catch (XsException& exception) {
        xsError << "An error occurred: " << exception.text();
        xmeTerminate();
        return 1;
    }

    // Clean up and exit
    xmeTerminate();
    return EXIT_SUCCESS;
}
