/* Copyright (c) Xsens Technologies B.V., 2006-2012. All rights reserved.

	  This source code is provided under the MT SDK Software License Agreement
and is intended for use only by Xsens Technologies BV and
	   those that have explicit written permission to use it from
	   Xsens Technologies BV.

	  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	   KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	   IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	   PARTICULAR PURPOSE.
 */

#pragma once

#include <xsens/xsresultvalue.h>
#include <xsens/xsbytearray.h>
#include <xsens/xsmessagearray.h>
#include <xsens/xsdeviceid.h>
#include <xsens/xsportinfo.h>
#include <xsens/xsoutputmode.h>
#include <xsens/xsoutputsettings.h>
#include <xsens/xsoutputconfigurationarray.h>

class StreamInterface;

class DeviceClass
{
public:
	DeviceClass(void);
	~DeviceClass(void);

	bool openPort(const XsPortInfo& portInfo);
	void close();

	XsResultValue readDataToBuffer(XsByteArray& raw);
	XsResultValue processBufferedData(XsByteArray& rawIn, XsMessageArray& messages);
	bool waitForMessage(XsXbusMessageId xmid, XsMessage& rcv);
	bool writeMessage(const XsMessage& msg);
	bool gotoConfig();
	bool gotoMeasurement();
	XsString getProductCode();
	XsDeviceId getDeviceId();
	bool setDeviceMode(const XsOutputMode& outputMode, const XsOutputSettings& outputSettings);
	bool setOutputConfiguration(XsOutputConfigurationArray& config);

private:
	StreamInterface *m_streamInterface;
	XsByteArray m_dataBuffer;
};

