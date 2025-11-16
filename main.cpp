#include "DSENT.h"
#include "model/Model.h"
#include <iostream>
#include <fstream>
#include <map>
#include <vector>

using namespace DSENT;
using namespace std;

const double WIDTH_MULTIPLIER = 6.0;
const size_t DATA_WIDTH = 32;
const size_t NUM_METAL_LAYERS = 2;

struct Metrics
{
    Metrics(double d = 0, double p = 0, double a = 0, double w = 0)
        : delay_s(d), power_j(p), area_m2(a), wireLength_m(w)
    {

    }

    double delay_s;
    double power_j;
    double area_m2;
    double wireLength_m;

    Metrics& operator+=(const Metrics& other) {
        delay_s += other.delay_s;
        power_j += other.power_j;
        area_m2 += other.area_m2;
        wireLength_m += other.wireLength_m;
        return *this;
    }
};

double wireLength(size_t numPorts,
                  size_t widthFactor,
                  size_t dataWidth = DATA_WIDTH,
                  size_t numMetalLayers = NUM_METAL_LAYERS)
{
    // ASSERT_PRINT(numPorts > 0, "numPorts must be > 0");
    // ASSERT_PRINT(dataWidth > 0, "dataWidth must be > 0");
    // ASSERT_PRINT(numMetalLayers > 0, "numMetalLayers must be > 0");

    const double N = static_cast<double>(numPorts);
    const double DW = static_cast<double>(dataWidth);
    const double WF = static_cast<double>(widthFactor);
    const double M = static_cast<double>(numMetalLayers);

    const double totalLength = (WF*  DW*  N) / M;

    return totalLength;
} // to be multiplies by wire pitch outside

std::vector<double> getBanyanLengthVector(size_t switchSize)
{
    std::vector<double> lengthVector;

    size_t numStages = static_cast<size_t>(std::log2(switchSize));
    //size_t N0 = 2; // each wire is connected to two outputs

    // input line calculations
    // double inputLinkLength = wireLength(N0, WIDTH_MULTIPLIER);
    // lengthVector.push_back(inputLinkLength);

    // inter-stage line calculations
    for (size_t stage = 0; stage < numStages; stage++)
    {
        // input line calculations
        size_t wireSpan = static_cast<size_t>(switchSize / std::pow(2, stage + 1));
        double L = wireLength(wireSpan, WIDTH_MULTIPLIER);
        lengthVector.push_back(L);
    }

    // output line calculations
    // double outputLinkLength = wireLength(switchSize, WIDTH_MULTIPLIER);
    // lengthVector.push_back(outputLinkLength);

    return lengthVector;
}

std::vector<double> getXbarLengthVector(size_t switchSize)
{
    std::vector<double> lengthVector;

    // input line calculations
    double inputLinkLength = wireLength(switchSize, WIDTH_MULTIPLIER);
    lengthVector.push_back(inputLinkLength);

    // output line calculations
    double outputLinkLength = wireLength(switchSize, WIDTH_MULTIPLIER);
    lengthVector.push_back(outputLinkLength);

    return lengthVector;
}

std::vector<double> getParadoxLengthVector(size_t switchSize)
{
    std::vector<double> lengthVector;
    double L = wireLength(switchSize/2, WIDTH_MULTIPLIER);
    lengthVector.push_back(L);

    return lengthVector;
}

std::vector<double> getHierXbarLengthVector(size_t switchSize)
{
    std::vector<double> lengthVector;
    double L = wireLength(switchSize, WIDTH_MULTIPLIER);
    lengthVector.push_back(L);
    //lengthVector.push_back(L);

    return lengthVector;
}

size_t subSwitchSize(size_t switchSize)
{
    size_t subSize = static_cast<size_t>(std::sqrt(switchSize));
    return subSize;
}

double reportDelay(const map<String, String>& params, Model* ms_model)
{
    const String& net_name = params.at("ReportTiming->StartNetNames");

    ElectricalModel* electrical_model = static_cast<ElectricalModel* >(ms_model);
    ElectricalTimingTree timing_tree(electrical_model->getInstanceName(), electrical_model);

    timing_tree.performCritPathExtract(electrical_model->getNet(net_name));
    double total_delay = timing_tree.calculateCritPathDelay(electrical_model->getNet(net_name));

    return total_delay;
}

Metrics evaluateInterconnectModel(const char* cfg_file, size_t connectedGates, bool repeatedLine, double wireLength, double widthMultiplier, ofstream& csv)
{
    map<String, String> config; 
    Model* model = initialize(cfg_file, config, wireLength, widthMultiplier, repeatedLine, connectedGates);

    const Result* area = static_cast<const Result* >(model->processQuery("Area", "Active"));
    const Result* power = static_cast<const Result* >(model->processQuery("NddPower", "Leakage"));
    double area_val = area ? area->calculateSum() : -1;
    double delay_val = reportDelay(config, model);
    double power_val = power ? power->calculateSum(): -1;
    finalize(config, model);
    std::cout << "----$ eval" << ": " << wireLength << ", "
        << "repeated: " << (repeatedLine ? "True" : "False") << ", "
        << "wireLength(m): " << wireLength << ", "
        << "wire pitch: " << model->getTechModel()->get("Wire->Global->MinSpacing").toDouble() << ", "
        << "connectedGates: " << connectedGates << ", "
        << "delay(s): " << delay_val << ", "
        << "power(J): " << power_val << ", "
        << "area(m^2): " << area_val << std::endl;
    return Metrics{delay_val, power_val, area_val, (wireLength * model->getTechModel()->get("Wire->Global->MinSpacing").toDouble())} ;
}

void writeCsvRow(std::ofstream& csv,
                 int switchType,
                 std::vector<std::string> switchTypes,
                 int switchSize,
                 const Metrics& m)
{
    csv << switchTypes[switchType] << ','
        << switchSize << ','
        << m.wireLength_m << ','
        << m.area_m2 << ','
        << m.delay_s << ','
        << m.power_j << ','
        << '\n';
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        cerr << "Usage: ./dsent <config_file.cfg>" << endl;
        return 1;
    }

    const char* cfg_file = argv[1];
    ofstream csv("repeated_link_sweep.csv");
    // generate csv header
    csv << "Switch type,Switch size,WireLength(m),Area(m^2),Delay(s),Power(J)\n";

    // switch params
    std::vector<std::string> switchTypes = {"Crossbar_centeralized", "Crossbar_distributed", "Banyan", "Paradox", "HierarchicalXbar"};
    std::vector<size_t> switchSizes = {4, 8, 16, 32, 64, 128, 256};
    std::vector<std::vector<Metrics>> allMetrics(switchTypes.size(), std::vector<Metrics>(switchSizes.size() + 1));

    for (int switchType = 0; switchType < static_cast<int>(switchTypes.size()); ++switchType)
    {
        unsigned int counter = 0;

        for (int size : switchSizes)
        {
            Metrics acc;

            std::vector<double> lengths;
            bool repeated = true;
            std::cout << "Evaluating switch type: " << switchTypes[switchType]  << " , size: " << size << std::endl;

            switch (switchType)
            {
            case 0: // Crossbar_centeralized
            {   
                lengths = getXbarLengthVector(size);
                for (auto len : lengths)
                    acc += evaluateInterconnectModel(cfg_file, size, repeated, len, WIDTH_MULTIPLIER, csv);
                break;
            }
            case 1: // Crossbar_distributed
            {
                lengths = getXbarLengthVector(size);
                if (lengths.size() >= 2)
                {
                    acc += evaluateInterconnectModel(cfg_file, size, true, lengths[0], WIDTH_MULTIPLIER, csv);
                    acc += evaluateInterconnectModel(cfg_file, size, false, lengths[1], WIDTH_MULTIPLIER, csv);
                }
                break;
            }
            case 2: // Banyan
            {
                lengths = getBanyanLengthVector(size);
                    for (auto len : lengths)
                        acc += evaluateInterconnectModel(cfg_file, 2, repeated, len, WIDTH_MULTIPLIER, csv);
                    break;
            }
            case 3: // Paradox
            {
                int subSize = subSwitchSize(size);
                Metrics sub = allMetrics[2][subSize];
                lengths = getParadoxLengthVector(size);
                for (auto len : lengths)
                    acc += evaluateInterconnectModel(cfg_file, 2, repeated, len, WIDTH_MULTIPLIER, csv);
                acc.delay_s += sub.delay_s;
                acc.power_j += sub.power_j;
                acc.area_m2 += sub.area_m2;
                break;
            }
            case 4: // HierarchicalXbar
            {
                int subSize = subSwitchSize(size);
                Metrics sub = allMetrics[1][subSize];
                lengths = getHierXbarLengthVector(size);
                for (auto len : lengths)
                    acc += evaluateInterconnectModel(cfg_file, subSize, repeated, len, WIDTH_MULTIPLIER, csv);
                acc.delay_s += sub.delay_s;
                acc.power_j += sub.power_j;
                acc.area_m2 += sub.area_m2;
                break;
            }
            }

            for (auto len : lengths)
                std::cout << "----$ Length vector: " << len << std::endl;

            allMetrics[switchType][counter] = acc;
            writeCsvRow(csv, switchType, switchTypes, size, acc);
            ++counter;
        }
    }

    csv.close();
    return 0;
}
