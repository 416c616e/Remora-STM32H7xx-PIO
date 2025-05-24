#ifndef COMMSINTERFACE_H
#define COMMSINTERFACE_H

#include <functional>

#include "../modules/module.h"

class CommsInterface : public Module {
private:

protected:
	std::function<void(bool)> dataCallback;

public:
	CommsInterface();

	virtual void init(void);
	virtual void start(void);
	virtual void tasks(void);

    void setDataCallback(const std::function<void(bool)>& callback) {
        dataCallback = callback;
    }
};

#endif
