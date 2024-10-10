#ifndef THREEDMAP
#define THREEDMAP


#include <iostream>
#include <unordered_map>
#include <string>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <algorithm>
#include <vector>
#include <filesystem>
#include <limits>


std::string extractFileName(const std::string& path)
{
    std::filesystem::path p = path;
    return p.stem().string(); 
}

struct Resolution
{
	enum Level
	{
		Invalid = -1,
		ANCHOR = 0,
		HIGH = 1,
		MID = 2,
		LOW = 3
	};
};

double decodeGvalue([[maybe_unused]]std::string valueSymbol)
{
    return 0;
};


class ThreeDMap 
{
private:
    std::unordered_map<std::string, std::string> map;
    int sizeX, sizeY, sizeZ; //e.g. x: [0,1,2,3] sizeX = 4;
    int minX, maxX, minY, maxY, minZ, maxZ; //e.g. minX=0, maxX=3;
    int resolutionScale;
    std::string mapName; 
    std::string generateKey(int x, int y, int z) 
    {
        return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(z);
    }

    std::string generateKey(const std::array<int,3> & position) 
    {
        return std::to_string(position[0]) + "_" + std::to_string(position[1]) + "_" + std::to_string(position[2]);
    }

    void initialMap(const std:: string & MapAddress)
    {      
        std::ifstream file(MapAddress);
        if (!file.is_open()) 
        {
            std::cerr << "Failed to open file for reading.\n";
        }

        std::string line;
    
        maxX = std::numeric_limits<int>::min();
        maxY = std::numeric_limits<int>::min();
        maxZ = std::numeric_limits<int>::min();
        minX = std::numeric_limits<int>::max();
        minY = std::numeric_limits<int>::max();
        minZ = std::numeric_limits<int>::max();

        while (std::getline(file, line)) {
            std::size_t start = line.find('(');
            std::size_t end = line.find(')', start);
            if (start != std::string::npos && end != std::string::npos) {
                std::istringstream iss(line.substr(start + 1, end - start - 1));
                int x, y, z;
                std::string value;
                char comma1, comma2, comma3;
                if (iss >> x >> comma1 >> y >> comma2 >> z >> comma3 >> value) {
                    if (comma1 == ',' && comma2 == ',' && comma3 == ',') 
                    {
                        std::string key = generateKey(x, y, z);
                        map[key] = value;
                        maxX = std::max(maxX, x);
                        maxY = std::max(maxY, y);
                        maxZ = std::max(maxZ, z);
                        minX = std::min(minX, x);
                        minY = std::min(minY, y);
                        minZ = std::min(minZ, z);
                    }
                }
            }
        }
    
        file.close();
        sizeX = maxX - minX + 1;
        sizeY = maxY - minY + 1;
        sizeZ = maxZ - minZ + 1;
    
    }
    

public:
    ThreeDMap(int sizeX, int sizeY, int sizeZ, int resolutionScale) : sizeX(sizeX), sizeY(sizeY), sizeZ(sizeZ), resolutionScale(resolutionScale)
    {
        maxX = std::numeric_limits<int>::min();
        maxY = std::numeric_limits<int>::min();
        maxZ = std::numeric_limits<int>::min();
        minX = std::numeric_limits<int>::max();
        minY = std::numeric_limits<int>::max();
        minZ = std::numeric_limits<int>::max();
        std::srand(std::time(nullptr)); // random seed
    }

    ThreeDMap() : sizeX(0), sizeY(0), sizeZ(0), resolutionScale(1) // for path
    {
        maxX = std::numeric_limits<int>::min();
        maxY = std::numeric_limits<int>::min();
        maxZ = std::numeric_limits<int>::min();
        minX = std::numeric_limits<int>::max();
        minY = std::numeric_limits<int>::max();
        minZ = std::numeric_limits<int>::max();
        std::srand(std::time(nullptr)); // random seed
    }

    ThreeDMap(const std:: string & MapAddress, int resolutionScale) : resolutionScale(resolutionScale), mapName(extractFileName(MapAddress)+"_path.map")          
    {

        initialMap(MapAddress);
        std::srand(std::time(nullptr)); // random seed
    }

    void generateRandomMap() 
    {
        for (int x = 0; x < sizeX; ++x) {
            for (int y = 0; y < sizeY; ++y) {
                for (int z = 0; z < sizeZ; ++z) {
                    std::string key = generateKey(x, y, z);
                    std::string value = (std::rand() % 100 < 20) ? "ob" : "non"; // 20%
                    map[key] = value;
                }
            }
        }
        maxX = sizeX -1;
        maxY = sizeY -1;
        maxZ = sizeZ -1;
        minX = 0;
        minY = 0;
        minZ = 0;
    }

    std::string getMapName()
    {
        return mapName; 
    }

    std::string getMapValue(int x, int y, int z) 
    {
        std::string key = generateKey(x, y, z);
        auto it = map.find(key);
        if (it != map.end()) 
        {
            return it->second;
        } 
        else 
        {
            return "non"; //without obstacle
        }
    }

    std::string getMapValue(const std::array<int,3> & position) 
    {
        std::string key = generateKey(position);
        auto it = map.find(key);
        if (it != map.end()) 
        {
            return it->second;
        } 
        else 
        {
            return "non"; //without obstacle
        }
    }

    void setMapValue(int x, int y, int z, const std::string& value)
    {
        bool withinBounds = checkBoundary(x,y,z);
        if (withinBounds)
        {
            std::string key = generateKey(x, y, z);
            map[key] = value;
        }
        else
        {
            std::cerr <<"Out of boundary!!!"<<std::endl;
        }
    }

    void addPath(int x, int y, int z, const std::string& value)
    {
        std::string key = generateKey(x, y, z);
        map[key] = value;
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
        maxZ = std::max(maxZ, z);
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        minZ = std::min(minZ, z);
        sizeX = maxX - minX + 1;
        sizeY = maxY - minY + 1;
        sizeZ = maxZ - minZ + 1;
    }

    void addPath(std::array<int,3>position, const std::string& value)
    {
        std::string key = generateKey(position);
        map[key] = value;
        maxX = std::max(maxX, position[0]);
        maxY = std::max(maxY, position[1]);
        maxZ = std::max(maxZ, position[2]);
        minX = std::min(minX, position[0]);
        minY = std::min(minY, position[1]);
        minZ = std::min(minZ, position[2]);
        sizeX = maxX - minX + 1;
        sizeY = maxY - minY + 1;
        sizeZ = maxZ - minZ + 1;
    }

    void removeMapValue(int x, int y, int z) 
    {
        std::string key = generateKey(x, y, z);
        map.erase(key);
    }

    bool checkBoundary(std::array<int,3> position)
    {
        //x
        if (position[0] < minX || position[0] > maxX) 
        {
            return false;
        }
        //y
        if (position[1] < minY || position[1] > maxY) 
        {
            return false;
        }
        //z
        if (position[2] < minZ || position[2] > maxZ) 
        {
            return false;
        }

        return true;
    }

    bool checkBoundary(const int & x, const int & y, const int & z)
    {
        //x
        if (x < minX || x > maxX) 
        {
            return false;
        }
        //y
        if (y < minY || y > maxY) 
        {
            return false;
        }
        //z
        if (z< minZ || z > maxZ) 
        {
            return false;
        }

        return true;
    }

    void printBoundary()
    {
        std::cout << "sizeX, sizeY, sizeZ: " << sizeX <<", "<< sizeY <<", "<< sizeZ << std::endl;
        std::cout << "minX, maxX: " << minX <<", "<< maxX << std::endl;
        std::cout << "minY, maxY: " << minY <<", "<< maxY << std::endl;
        std::cout << "minZ, maxZ: " << minZ <<", "<< maxZ << std::endl;

    }

    std::vector<std::pair<std::array<int,3>,std::string>> getSuccs(const std::array<int,3> & position, Resolution::Level res)
    {
        if (res == Resolution::Invalid)
        {
            std::cerr << "Invalid resoluion" << std::endl;
            return {};
        }
        int resFactor = resolutionScale * res;  
        //std::cout<<"resFactor of "<<res<<": "<<resFactor<<std::endl;
        std::vector<std::pair<std::array<int, 3>,std::string>> expandableNodes;
        std::array<int, 3> map_size = {sizeX, sizeY, sizeZ};
        std::array<int, 3> directions = {{resFactor*0, resFactor*(-1), resFactor*(1)}};  // Only two directions, -1 and 1, for each dimension.

        for (int x : directions) 
        {
            for (int y : directions) 
            {
                for (int z : directions) 
                {
                    if (x == 0 && y == 0 && z == 0) continue; 
                    std::array<int, 3> neighbor = position;
                    bool skip = false;
                    if (map_size[0] != 0) neighbor[0] += x; else if (x != 0) skip = true;
                    if (map_size[1] != 0) neighbor[1] += y; else if (y != 0) skip = true;
                    if (map_size[2] != 0) neighbor[2] += z; else if (z != 0) skip = true;
                    if (skip) continue; 

                    bool withinBounds = checkBoundary(neighbor);
                    std::string value = getMapValue(neighbor);

                    if (value != "ob"&& withinBounds) 
                    {
                        expandableNodes.emplace_back(neighbor,value);
                    }
                    else if(!withinBounds)
                    {
                        std::cout<< "Out of the range of map!!!"<<std::endl;
                    }
                }
            }
        }
    //std::cout<<expandableNodes.size()<<std::endl;
    return expandableNodes;
    }
    
    
    std::vector<std::pair<std::array<int,3>,std::string>> getSuccs(const std::array<int,3> & position)
    {
        std::vector<std::pair<std::array<int, 3>,std::string>> expandableNodes;
        std::array<int, 3> map_size = {sizeX, sizeY, sizeZ};
        std::array<int, 3> directions = {{0, -1, 1}};  // Only two directions, -1 and 1, for each dimension.

        for (int x : directions) 
        {
            for (int y : directions) 
            {
                for (int z : directions) 
                {
                    if (x == 0 && y == 0 && z == 0) continue; // skip current position
                    std::array<int, 3> neighbor = position;
                    bool skip = false;
                    // direction of expansion (map_size)
                    if (map_size[0] != 0) neighbor[0] += x; else if (x != 0) skip = true;
                    if (map_size[1] != 0) neighbor[1] += y; else if (y != 0) skip = true;
                    if (map_size[2] != 0) neighbor[2] += z; else if (z != 0) skip = true;
                    if (skip) continue; 
                    bool withinBounds = checkBoundary(neighbor);
                    std::string value = getMapValue(neighbor);

                    if (value != "ob"&& withinBounds) // Not expand obstcale, can add safty check !!!
                    {
                        expandableNodes.emplace_back(neighbor,value);
                    }
                }
            }
        }

    return expandableNodes;
    }
    
    void printNodes(const std::vector<std::pair<std::array<int,3>,std::string>>& nodes) 
    {
        for (const auto& node : nodes)
        {
            std::cout << "(" << node.first[0] << ", " << node.first[1] << ", " << node.first[2] << ")" << std::endl;
        }
    }



    void saveToFile(const std::string& filename) const // path, extension list...
    {
        std::ofstream file(filename);
        if (!file.is_open()) 
        {
            std::cerr << "Failed to open file for writing.\n";
            return;
        }
        // Titles:
        file << "Map_Name: " << mapName << std::endl;

        for (auto& entry : map) 
        {
            std::string key = entry.first; //coordinate "x_y_z"
            std::string value = entry.second; // value
            int x, y, z;
            sscanf(key.c_str(), "%d_%d_%d", &x, &y, &z); // Parse the key back to coordinates
            file << "(" << x << ", " << y << ", " << z << ", " << value << ")\n";
        }

        file.close();
    }
};

#endif //THREEDMAP