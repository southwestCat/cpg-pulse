#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Tools/Module/Blackboard.h"

class GroundContactStateProvider
{
public:
    void update(GroundContactState &g);

private:
    REQUIRES_REPRESENTATION(FsrSensorData);
};
