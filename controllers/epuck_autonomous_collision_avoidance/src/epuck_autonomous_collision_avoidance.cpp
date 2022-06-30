#include "MainControllerEPuck.h"

int main(int argc, char **argv) {

    /* create instance of robot main controller */
    IMainController* robotMainController = new MainControllerEPuck();

    /* call main routine of robot controller */
    robotMainController->mainControllerRoutine(argc, argv);
        
    delete robotMainController;

    return 0;
}
