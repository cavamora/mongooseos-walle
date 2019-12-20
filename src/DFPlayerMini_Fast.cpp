#include "DFPlayerMini_Fast.h"
#include "mgos.h"



bool DFPlayerMini_Fast::begin(int uart_no)
{
	_uart_no = uart_no;

	sendStack.start_byte = SB;
	sendStack.version    = VER;
	sendStack.length     = LEN;
	sendStack.end_byte   = EB;

	recStack.start_byte  = SB;
	recStack.version     = VER;
	recStack.length      = LEN;
	recStack.end_byte    = EB;

	return true;
}




void DFPlayerMini_Fast::playNext()
{
	sendStack.commandValue  = NEXT;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::playPrevious()
{
	sendStack.commandValue  = PREV;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::play(uint16_t trackNum)
{
	sendStack.commandValue  = PLAY;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = (trackNum >> 8) & 0xFF;
	sendStack.paramLSB = trackNum & 0xFF;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::incVolume()
{
	sendStack.commandValue = INC_VOL;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::decVolume()
{
	sendStack.commandValue = DEC_VOL;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::volume(uint8_t volume)
{
	if (volume <= 30)
	{
		sendStack.commandValue = VOLUME;
		sendStack.feedbackValue = NO_FEEDBACK;
		sendStack.paramMSB = 0;
		sendStack.paramLSB = volume;

		findChecksum(&sendStack);
		sendData();
	}
}




void DFPlayerMini_Fast::EQSelect(uint8_t setting)
{
	if (setting <= 5)
	{
		sendStack.commandValue = EQ;
		sendStack.feedbackValue = NO_FEEDBACK;
		sendStack.paramMSB = 0;
		sendStack.paramLSB = setting;

		findChecksum(&sendStack);
		sendData();
	}
}




void DFPlayerMini_Fast::loop(uint16_t trackNum)
{
  sendStack.commandValue = PLAYBACK_MODE;
  sendStack.feedbackValue = NO_FEEDBACK;
  sendStack.paramMSB = (trackNum >> 8) & 0xFF;
  sendStack.paramLSB = trackNum & 0xFF;
  
  findChecksum(&sendStack);
  sendData();
}




void DFPlayerMini_Fast::playbackSource(uint8_t source)
{
	if ((source > 0) && (source <= 5))
	{
		sendStack.commandValue  = PLAYBACK_SRC;
		sendStack.feedbackValue = NO_FEEDBACK;
		sendStack.paramMSB = 0;
		sendStack.paramLSB = source;

		findChecksum(&sendStack);
		sendData();
	}
}




void DFPlayerMini_Fast::standbyMode()
{
	sendStack.commandValue  = STANDBY;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::normalMode()
{
	sendStack.commandValue  = NORMAL;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::reset()
{
	sendStack.commandValue  = RESET;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::resume()
{
	sendStack.commandValue  = PLAYBACK;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::pause()
{
	sendStack.commandValue  = PAUSE;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::playFolder(uint8_t folderNum, uint8_t trackNum)
{
	sendStack.commandValue  = SPEC_FOLDER;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = folderNum;
	sendStack.paramLSB = trackNum;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::volumeAdjustSet(uint8_t gain)
{
	if (gain <= 31)
	{
		sendStack.commandValue  = VOL_ADJ;
		sendStack.feedbackValue = NO_FEEDBACK;
		sendStack.paramMSB = 0;
		sendStack.paramLSB = VOL_ADJUST + gain;

		findChecksum(&sendStack);
		sendData();
	}
}




void DFPlayerMini_Fast::startRepeatPlay()
{
	sendStack.commandValue  = REPEAT_PLAY;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = START_REPEAT;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::stopRepeatPlay()
{
	sendStack.commandValue  = REPEAT_PLAY;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = STOP_REPEAT;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::playFromMP3Folder(uint16_t trackNum)
{
	sendStack.commandValue  = SPEC_TRACK_3000;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = (trackNum >> 8) & 0xFF;
	sendStack.paramLSB = trackNum & 0xFF;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::repeatFolder(uint16_t folder)
{
	sendStack.commandValue  = REPEAT_FOLDER;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = (folder >> 8) & 0xFF;
	sendStack.paramLSB = folder & 0xFF;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::randomAll()
{
	sendStack.commandValue  = RANDOM_ALL;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 0;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::startRepeat()
{
	sendStack.commandValue  = REPEAT_CURRENT;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 0;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::stopRepeat()
{
	sendStack.commandValue  = REPEAT_CURRENT;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::startDAC()
{
	sendStack.commandValue  = SET_DAC;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 0;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::stopDAC()
{
	sendStack.commandValue  = SET_DAC;
	sendStack.feedbackValue = NO_FEEDBACK;
	sendStack.paramMSB = 0;
	sendStack.paramLSB = 1;

	findChecksum(&sendStack);
	sendData();
}




void DFPlayerMini_Fast::sleep()
{
	playbackSource(SLEEP);
}




void DFPlayerMini_Fast::wakeUp()
{
	playbackSource(TF);

	delay(100);
}




bool DFPlayerMini_Fast::isPlaying()
{
	int16_t result = query(GET_STATUS);

	if (result != -1)
		return (result & 1);

	return result;
}




int16_t DFPlayerMini_Fast::currentVolume()
{
	return query(GET_VOL);
}




int16_t DFPlayerMini_Fast::currentEQ()
{
	return query(GET_EQ);
}




int16_t DFPlayerMini_Fast::currentMode()
{
	return query(GET_MODE);
}




int16_t DFPlayerMini_Fast::currentVersion()
{
	return query(GET_VERSION);
}




int16_t DFPlayerMini_Fast::numUsbTracks()
{
	return query(GET_TF_FILES);
}




int16_t DFPlayerMini_Fast::numSdTracks()
{
	return query(GET_U_FILES);
}




int16_t DFPlayerMini_Fast::numFlashTracks()
{
	return query(GET_FLASH_FILES);
}




int16_t DFPlayerMini_Fast::currentUsbTrack()
{
	return query(GET_TF_TRACK);
}




int16_t DFPlayerMini_Fast::currentSdTrack()
{
	return query(GET_U_TRACK);
}




int16_t DFPlayerMini_Fast::currentFlashTrack()
{
	return query(GET_FLASH_TRACK);
}




int16_t DFPlayerMini_Fast::numTracksInFolder(uint8_t folder)
{
	return query(GET_FOLDER_FILES, (folder >> 8) & 0xFF, folder & 0xFF);
}




int16_t DFPlayerMini_Fast::numFolders()
{
	return query(GET_FOLDERS);
}




void DFPlayerMini_Fast::findChecksum(stack *_stack)
{
	uint16_t checksum = (~(_stack->version + _stack->length + _stack->commandValue + _stack->feedbackValue + _stack->paramMSB + _stack->paramLSB)) + 1;

	_stack->checksumMSB = checksum >> 8;
	_stack->checksumLSB = checksum & 0xFF;
}




void DFPlayerMini_Fast::sendData()
{
	mgos_uart_write(_uart_no, &sendStack.start_byte, 1);
	mgos_uart_write(_uart_no, &sendStack.version, 1);
	mgos_uart_write(_uart_no, &sendStack.length, 1);
	mgos_uart_write(_uart_no, &sendStack.commandValue, 1);
	mgos_uart_write(_uart_no, &sendStack.feedbackValue, 1);
	mgos_uart_write(_uart_no, &sendStack.paramMSB, 1);
	mgos_uart_write(_uart_no, &sendStack.paramLSB, 1);
	mgos_uart_write(_uart_no, &sendStack.checksumMSB, 1);
	mgos_uart_write(_uart_no, &sendStack.checksumLSB, 1);
	mgos_uart_write(_uart_no, &sendStack.end_byte, 1);
	mgos_uart_flush(_uart_no);
}




void DFPlayerMini_Fast::flush()
{
	char temp;
	while (mgos_uart_read_avail(_uart_no)) {
		mgos_uart_read(_uart_no, &temp, 1);
	}
		
}




int16_t DFPlayerMini_Fast::query(uint8_t cmd, uint8_t msb, uint8_t lsb)
{
	flush();

	sendStack.commandValue  = cmd;
	sendStack.feedbackValue = FEEDBACK;
	sendStack.paramMSB = msb;
	sendStack.paramLSB = lsb;

	findChecksum(&sendStack);
	sendData();

	if (getStatus(cmd))
		return (recStack.paramMSB << 8) | recStack.paramLSB;

	return -1;
}




bool DFPlayerMini_Fast::getStatus(uint8_t cmd)
{
	timer = millis();
	bool result = parseFeedback();

	if (!result)
		return false;

	while (recStack.commandValue != cmd)
	{
		if (timeout())
			return false;

		result = parseFeedback();

		if (!result)
			return false;
	}

	return true;
}




bool DFPlayerMini_Fast::parseFeedback()
{
	while (mgos_uart_read_avail(_uart_no) < STACK_SIZE)
		if (timeout())
			return false;

	mgos_uart_read(_uart_no, &recStack.start_byte, 1);
	mgos_uart_read(_uart_no, &recStack.start_byte, 1);
	mgos_uart_read(_uart_no, &recStack.version, 1);
	mgos_uart_read(_uart_no, &recStack.length, 1);
	mgos_uart_read(_uart_no, &recStack.commandValue, 1);
	mgos_uart_read(_uart_no, &recStack.feedbackValue, 1);
	mgos_uart_read(_uart_no, &recStack.paramMSB, 1);
	mgos_uart_read(_uart_no, &recStack.paramLSB, 1);
	mgos_uart_read(_uart_no, &recStack.checksumMSB, 1);
	mgos_uart_read(_uart_no, &recStack.checksumLSB, 1);
	mgos_uart_read(_uart_no, &recStack.end_byte, 1);

	return true;
}




bool DFPlayerMini_Fast::timeout()
{
	if ((millis() - timer) >= threshold)
		return true;
	return false;
}



/*
void DFPlayerMini_Fast::printStack(stack _stack)
{
	LOG(LL_INFO, ("Stack:"));
	LOG(LL_INFO, (_stack.start_byte, HEX));
	LOG(LL_INFO, (_stack.version, HEX));
	LOG(LL_INFO, (_stack.length, HEX));
	LOG(LL_INFO, (_stack.commandValue, HEX));
	LOG(LL_INFO, (_stack.feedbackValue, HEX));
	LOG(LL_INFO, (_stack.paramMSB, HEX));
	LOG(LL_INFO, (_stack.paramLSB, HEX));
	LOG(LL_INFO, (_stack.checksumMSB, HEX));
	LOG(LL_INFO, (_stack.checksumLSB, HEX));
	LOG(LL_INFO, (_stack.end_byte, HEX));
}
*/