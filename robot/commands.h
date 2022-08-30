#ifndef COMMANDS_H
#define COMMANDS_H

enum class CommandName
{
    Forward,
    Backward,
    TurnLeft,
    TurnRight,
};

class Command
{
private:
    static bool isLineDetected(int duration_ms, bool isStoppable);

public:
    CommandName name;
    int value;
    bool isStoppable;
    static void dump(const Command commands[], int length);
    static void run(const Command commands[], int length);
};

#endif