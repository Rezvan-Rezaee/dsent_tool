#include "DSENT.h"
#include "model/Model.h"
#include <iostream>
#include <map>
using namespace DSENT;
using namespace std;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        cerr << "Usage: ./dsent <config_file.cfg>" << endl;
        return 1;
    }

    const char *cfg_file = argv[1];
    map<String, String> config;

    // Initialize DSENT with config file
    Model *model = initialize(cfg_file, config);
    std::cout << "*** Finished constructing the model." << std::endl;

    // Evaluate key metrics
    std::string queries = "Area";
    vector<String> outputs;

    const Result *area_result = static_cast<const Result *>(
        model->processQuery("Area", "Active"));

    std::cout << "Area (Active): " << area_result->calculateSum() << std::endl;

    // Cleanup
    finalize(config, model);
    return 0;
}