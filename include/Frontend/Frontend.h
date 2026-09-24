#pragma once

class Frontend
{
public:
    virtual ~Frontend() = default;

    virtual void init()=0;
    virtual void proces()=0;
    virtual void shutdown()=0;
};