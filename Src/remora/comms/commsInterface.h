#ifndef COMMSINTERFACE_H
#define COMMSINTERFACE_H

#include <functional>

#include "../modules/module.h"

class CommsInterface : public Module {
private:

protected:
    uint32_t rxCount;
    uint32_t txCount;

	std::function<void(bool)> dataCallback;

public:
	CommsInterface();

	virtual void init(void);
	virtual void start(void);
	virtual void tasks(void);

    void setDataCallback(const std::function<void(bool)>& callback) {
        dataCallback = callback;
    }

    uint32_t getRxCount() { return rxCount; }
    uint32_t getTxCount() { return txCount; }

	void incRxCount() { rxCount++; if (rxCount > 1000000) rxCount = 0; }
	void incTxCount() { txCount++; if (txCount > 1000000) txCount = 0; }

	void resetRxCount() { rxCount = 0; }
	void resetTxCount() { txCount = 0; }
};

#endif
