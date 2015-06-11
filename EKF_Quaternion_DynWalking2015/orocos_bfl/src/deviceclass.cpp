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

#include "deviceclass.h"
#include <xcommunication/protocolhandler.h>
#include <xcommunication/usbinterface.h>
#include <xcommunication/serialinterface.h>
#include <xcommunication/streaminterface.h>

DeviceClass::DeviceClass(void)
{
}

DeviceClass::~DeviceClass(void)
{
	if (m_streamInterface)
		delete m_streamInterface;
}

/*! \brief Open an IO device
	\param portInfo The info to use for opening the port
	\return True when successful
*/
bool DeviceClass::openPort(const XsPortInfo& portInfo)
{
	if (portInfo.isUsb())
		m_streamInterface = new UsbInterface();
	else
		m_streamInterface = new SerialInterface();

	if (m_streamInterface->open(portInfo) != XRV_OK)
		return false;
	return true;
}

/*! \brief Close an IO device
*/
void DeviceClass::close()
{
	m_streamInterface->close();
}

/*! \brief Read available data from the open IO device
	\details This function will attempt to read all available data from the open device (COM port
	or USB port).
	The function will read from the device, but it won't wait for data to become available.
	\param raw A XsByteArray to where the read data will be stored.
	\return Whether data has been read from the IO device
*/
XsResultValue DeviceClass::readDataToBuffer(XsByteArray& raw)
{
	// always read data and append it to the cache before doing analysis
	const int maxSz = 8192;
	XsResultValue res = m_streamInterface->readData(maxSz, raw);
	if (raw.size())
		return XRV_OK;

	return res;
}

/*! \brief Read all messages from the buffered read data after adding new data supplied in \a rawIn
	\details This function will read all present messages in the read buffer. In order for this function
	to work, you need to call readDataToBuffer() first.
	\param rawIn The buffered data in which to search for messages
	\param messages The messages found in the data
	\return The messages that were read.
*/
XsResultValue DeviceClass::processBufferedData(XsByteArray& rawIn, XsMessageArray& messages)
{
	ProtocolHandler protocol;

	if (rawIn.size())
		m_dataBuffer.append(rawIn);

	int popped = 0;
	messages.clear();

	for(;;)
	{
		XsByteArray raw(m_dataBuffer.data()+popped, m_dataBuffer.size()-popped);
		XsMessage message;
		MessageLocation location = protocol.findMessage(message, raw);

		if (location.isValid())
		{
			// message is valid, remove data from cache
			popped += location.m_size + location.m_startPos;
			messages.push_back(message);
		}
		else
		{
			if (popped)
				m_dataBuffer.pop_front(popped);

			if (messages.empty())
				return XRV_TIMEOUTNODATA;

			return XRV_OK;
		}
	}
}

/*! \brief Wait for the requested XsXbusMessageId
	\param xmid The message id to wait for
	\param rcv  The received message
	\return Whether the requested message was found
*/
bool DeviceClass::waitForMessage(XsXbusMessageId xmid, XsMessage& rcv)
{
	XsByteArray data;
	XsMessageArray msgs;
	bool foundAck = false;
	do
	{
		readDataToBuffer(data);
		processBufferedData(data, msgs);
		for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
			if ((*it).getMessageId() == xmid)
			{
				foundAck = true;
				rcv = *it;
			}
	} while (!foundAck);
	return foundAck;
}

/*! \brief Write a message to the IO device
	\param msg The message to write
	\return Whether the message could be written
*/
bool DeviceClass::writeMessage(const XsMessage& msg)
{
	XsByteArray raw;
	if (ProtocolHandler::composeMessage(raw, msg) < 0)
		return false;

	return (m_streamInterface->writeData(raw) == XRV_OK);
}

/*! \brief Put a device in config mode
	\return True when the device acknowledged config mode
*/
bool DeviceClass::gotoConfig()
{
	XsMessage snd(XMID_GotoConfig, 0), rcv;
	writeMessage(snd);

	return waitForMessage(XMID_GotoConfigAck, rcv);
}

/*! \brief Put a device in measurement mode
	\return True when the device acknowledged measurement mode
*/
bool DeviceClass::gotoMeasurement()
{
	XsMessage snd(XMID_GotoMeasurement, 0), rcv;
	writeMessage(snd);

	return waitForMessage(XMID_GotoMeasurementAck, rcv);
}

/*! \brief Request the product code from a device
	\return The product code when ok, otherwise an empty XsString
*/
XsString DeviceClass::getProductCode()
{
	XsMessage snd(XMID_ReqProductCode, 0), rcv;
	writeMessage(snd);

	if (waitForMessage(XMID_ProductCode,rcv))
	{
		const char* pc = (const char*) rcv.getDataBuffer(0);
		std::string result(pc?pc:"", rcv.getDataSize());
		std::string::size_type thingy = result.find(" ");
		if (thingy < 20)
			result.erase(result.begin() + thingy, result.end());	//lint !e534
		return XsString(result);
	}
	else
		return XsString();
}

/*! \brief Request the device id from a device
	\return The device id (XsDeviceId) when ok, otherwise an empty XsDeviceId
*/
XsDeviceId DeviceClass::getDeviceId()
{
	XsMessage snd(XMID_ReqDid, 0), rcv;
	writeMessage(snd);

	if (waitForMessage(XMID_DeviceId,rcv))
	{
		return rcv.getDataLong();
	}
	else
		return XsDeviceId();
}

/*! \brief Set the device mode of a device (outputmode and outputsettings)
	\param outputMode The XsOutputMode to set
	\param outputSettings The XsOutputSettings to set
	\return True when successful
*/
bool DeviceClass::setDeviceMode(const XsOutputMode& outputMode, const XsOutputSettings& outputSettings)
{
	XsMessage sndOM(XMID_SetOutputMode), sndOS(XMID_SetOutputSettings), rcv;

	sndOM.resizeData(2);
	sndOM.setDataShort((uint16_t) outputMode);
	writeMessage(sndOM);
	if (!waitForMessage(XMID_SetOutputModeAck, rcv))
		return false;

	XsMessage snd(XMID_SetOutputSettings);
	snd.resizeData(4);
	snd.setDataLong((uint32_t)outputSettings);
	writeMessage(sndOS);
	if (!waitForMessage(XMID_SetOutputSettingsAck, rcv))
		return false;

	return true;
}

/*! \brief Set the output configuration of a device
	\param config An array XsOutputConfigurationArray) containing the one or multiple XsOutputConfigurations
	\return True when successful
*/
bool DeviceClass::setOutputConfiguration(XsOutputConfigurationArray& config)
{
	XsMessage snd(XMID_SetOutputConfiguration, 4), rcv;
	if (config.size() == 0)
	{
		snd.setDataShort((uint16_t)XDI_None, 0);
		snd.setDataShort(0, 2);
	}
	else
	{
		for (XsSize i = 0; i < (XsSize) config.size(); ++i)
		{
			snd.setDataShort((uint16_t)config[i].m_dataIdentifier, i*4);
			snd.setDataShort(config[i].m_frequency, i*4+2);
		}
	}
	writeMessage(snd);

	return waitForMessage(XMID_SetOutputConfigurationAck, rcv);
}
