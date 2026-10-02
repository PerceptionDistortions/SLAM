#pragma once

class Frontend
{
public:
    virtual ~Frontend() = default;

    virtual bool init()=0;
    virtual void shutdown()=0;
};