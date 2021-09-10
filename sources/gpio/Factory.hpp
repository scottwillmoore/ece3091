#pragma once

class Factory {
public:
    static Factory& getFactory();

private:
    Factory();
    ~Factory();
};
