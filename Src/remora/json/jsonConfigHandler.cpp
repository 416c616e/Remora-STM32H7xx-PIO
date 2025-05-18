
#include "jsonConfigHandler.h"
#include "../remora.h"


JsonConfigHandler::JsonConfigHandler(Remora* _remora) :
	remoraInstance(_remora),
	configError(false)
{
	loadConfiguration();
    updateThreadFreq();
}

bool JsonConfigHandler::loadConfiguration() {
	// Clear any existing configuration
    jsonContent.clear();
    doc.clear();

    // Read and parse the configuration file
    if (!readFileContents()) {
        return false;
    }
    parseJson();
    return true;
}


void JsonConfigHandler::updateThreadFreq() {
    if (configError) return;

    JsonArray Threads = doc["Threads"];

    // create objects from JSON data
    for (JsonArray::iterator it=Threads.begin(); it!=Threads.end(); ++it) {
        thread = *it;
        const char* configor = thread["Thread"];
        uint32_t    freq = thread["Frequency"];
        if (!strcmp(configor,"Base")) {
        	printf("Updating thread frequency - Setting BASE thread frequency to %lu\n\r", freq);
            remoraInstance->setBaseFreq(freq);
        }
        else if (!strcmp(configor,"Servo")) {
            printf("Updating thread frequency - Setting SERVO thread frequency to %lu\n\r", freq);
            remoraInstance->setServoFreq(freq);
        }
    }
}

JsonArray JsonConfigHandler::getModules() {
	if (doc["Modules"].is<JsonVariant>())
        return doc["Modules"].as<JsonArray>();
    else
        return JsonArray();
}

bool JsonConfigHandler::readFileContents() {

	uint32_t bytesread; // bytes read count

    printf("Reading JSON configuration file\n\r");

    // Try to mount the file system
    printf("Mounting the file system... \n\r");
    if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
	{
    	printf("Failed to mount SD card\n\r");
    	Error_Handler();
	}
    else
    {
		//Open file for reading
		if(f_open(&SDFile, filename, FA_READ) != FR_OK)
		{
			printf("Failed to open JSON config file\n\n\r");
			Error_Handler();
		}
		else
		{
			int32_t length = f_size(&SDFile);
			printf("JSON config file lenght = %2ld\n\r", length);

			__attribute__((aligned(32))) char rtext[length];
			if(f_read(&SDFile, rtext, length, (UINT *)&bytesread) != FR_OK)
			{
				printf("JSON config file read FAILURE\n\n\r");
			}
			else
			{
				printf("JSON config file read SUCCESS!\n\n\r");
				// put JSON char array into std::string
				jsonContent.reserve(length + 1);
			    for (int i = 0; i < length; i++) {
			    	jsonContent = jsonContent + rtext[i];
			    }

			    // Remove comments from next line to print out the JSON config file
			    //printf("%s\n\r", jsonContent.c_str());
			}

			f_close(&SDFile);
		}
    }

	return true;
}


bool JsonConfigHandler::parseJson() {
	
	printf("Parsing JSON configuration file\n\r");
	
    // Clear any existing parsed data
    doc.clear();

    // Parse JSON
    DeserializationError error = deserializeJson(doc, jsonContent.c_str());

    printf("Config deserialisation - ");

    switch (error.code())
    {
        case DeserializationError::Ok:
            printf("Deserialization succeeded\n\n\r");
            break;
        case DeserializationError::InvalidInput:
            printf("Invalid input!\n\r");
            configError = true;
            break;
        case DeserializationError::NoMemory:
            printf("Not enough memory\\nn");
            configError = true;
            break;
        default:
            printf("Deserialization failed: ");
            printf(error.c_str());
            printf("\n\r");
            configError = true;
            break;
    }

    return true;
}
