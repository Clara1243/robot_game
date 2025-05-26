#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <ctime>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
using namespace std;

ofstream logFile;
void logAction(const string &message)
{
    if (logFile.is_open())
    {
        logFile << message << endl;
    }
}

class Battlefield;
class GenericRobot;

class Robot
{
public:
    enum Action
    {
        THINK,
        MOVE,
        SEE,
        SHOOT
    };

    enum UpgradeArea
    {
        MOVING,
        SHOOTING,
        SEEING
    };

protected:
    string name;
    int row;
    int col;
    char symbol;
    int lives;
    int shells;
    vector<UpgradeArea> chosenUpgrades;

public:
    Robot(const string &robotName, int r, int c);
    virtual ~Robot();

    string getName() const;
    int getRow() const;
    int getCol() const;
    char getSymbol() const;

    int getLives() const;
    void loseLives();
    bool isAlive() const;

    void destroy();

    int getShells() const;
    void useShell();

    virtual Action chooseAction();
    void performAction();

    virtual void think() = 0;
    virtual void move() = 0;
    virtual void see() = 0;
    virtual bool shoot() = 0;

    bool canUpgrade(UpgradeArea area) const;
    void applyUpgrade(UpgradeArea area);
    static string getAreaName(UpgradeArea area);
    // Method to trigger upgrade after destroying an enemy
    void triggerUpgradeChoice();
    // Add method to set position (for moving away after combat)
    void setPosition(int r, int c);
};

class Battlefield
{
private:
    int rows, cols, steps;
    vector<vector<char>> grid;
    vector<unique_ptr<Robot>> robots;

public:
    Battlefield();

    int getRows() const { return rows; }
    int getCols() const { return cols; }

    bool readFile(const string &filename);
    void updateGrid();
    void display() const;
    Robot *findTargetAt(int row, int col, Robot *attacker);
    bool handleCombat(Robot *attacker);
    bool positionOccupied(int r, int c);
    void lookAround(Robot *robot, int dx, int dy);
    void handleCollisions();
    void simulate();

    // Helper method for scout vision
    vector<Robot *> getAllAliveRobots() const
    {
        vector<Robot *> aliveRobots;
        for (const auto &robot : robots)
        {
            if (robot->isAlive())
            {
                aliveRobots.push_back(robot.get());
            }
        }
        return aliveRobots;
    }

    // Helper method to find nearby robots for tracker planting
    Robot *findNearestRobot(Robot *searcher) const
    {
        Robot *nearest = nullptr;
        int minDistance = INT_MAX;

        for (const auto &robot : robots)
        {
            if (robot.get() != searcher && robot->isAlive())
            {
                int distance = abs(robot->getRow() - searcher->getRow()) +
                               abs(robot->getCol() - searcher->getCol());
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearest = robot.get();
                }
            }
        }
        return nearest;
    }
};

// Battlefield Implementation
Battlefield::Battlefield() : rows(0), cols(0), steps(0) {}

void Battlefield::updateGrid()
{
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            grid[i][j] = '.';
        }
    }

    for (const auto &robot : robots)
    {
        if (robot->isAlive())
        {
            int r = robot->getRow();
            int c = robot->getCol();
            if (r >= 0 && r < rows && c >= 0 && c < cols)
            {
                grid[r][c] = robot->getSymbol();
            }
        }
    }
}

void Battlefield::display() const
{
    cout << "\nBattlefield:\n";
    cout << '+';
    for (int i = 0; i < cols; i++)
        cout << '-';
    cout << "+\n";

    for (const auto &row : grid)
    {
        cout << '|';
        for (char cell : row)
            cout << cell;
        cout << "|\n";
    }

    cout << '+';
    for (int i = 0; i < cols; i++)
        cout << '-';
    cout << "+\n";

    cout << "\nRobot Details:\n";
    for (const auto &robot : robots)
    {
        if (robot->isAlive())
        {
            cout << robot->getName()
                 << " (Lives:" << robot->getLives()
                 << ", Shells: " << robot->getShells() << ")"
                 << " at (" << robot->getRow() << "," << robot->getCol() << ")\n";
        }
    }

    // Now mirror it into the log file
    // (we build each line just once so console + log stay in sync)
    logAction("Battlefield:");
    // top border
    {
        string border = "+" + string(cols, '-') + "+";
        logAction(border);
    }
    // each row
    for (const auto &row : grid)
    {
        string line = "|";
        for (char cell : row)
            line += cell;
        line += "|";
        logAction(line);
    }
    // bottom border
    {
        string border = "+" + string(cols, '-') + "+";
        logAction(border);
    }
    // robot details
    logAction("Robot Details:");
    for (const auto &robot : robots)
    {
        if (robot->isAlive())
        {
            string detail =
                robot->getName() +
                " (Lives:" + to_string(robot->getLives()) +
                ", Shells:" + to_string(robot->getShells()) + ")" +
                " at (" + to_string(robot->getRow()) +

                "," + to_string(robot->getCol()) + ")";
            logAction(detail);
        }
    }
}

Robot *Battlefield::findTargetAt(int row, int col, Robot *attacker)
{
    for (const auto &robot : robots)
    {
        if (robot.get() != attacker && robot->isAlive() &&
            robot->getRow() == row && robot->getCol() == col)
        {
            return robot.get();
        }
    }
    return nullptr;
}

bool Battlefield::handleCombat(Robot *attacker)
{
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    string directions[] = {"left up", "up", "right up", "left", "right", "left down", "down", "right down"};

    int direction = rand() % 8;
    int targetRow = attacker->getRow() + dx[direction];
    int targetCol = attacker->getCol() + dy[direction];

    int shellsBefore = attacker->getShells();
    attacker->useShell();

    cout << attacker->getName() << " shoots " << directions[direction];

    string logMsg = "SHOOT: " + attacker->getName() + " shoots " + directions[direction] +
                    " from (" + to_string(attacker->getRow()) + "," + to_string(attacker->getCol()) + ")";

    if (targetRow >= 0 && targetRow < rows && targetCol >= 0 && targetCol < cols)
    {
        cout << " at position (" << targetRow << "," << targetCol << ")";
        logMsg += " targeting (" + to_string(targetRow) + "," + to_string(targetCol) + ")";

        Robot *target = findTargetAt(targetRow, targetCol, attacker);
        if (target)
        {
            if (rand() % 10 < 7)
            {
                cout << " - [HIT] " << target->getName() << " is hit!" << endl;
                logMsg += " - HIT " + target->getName() + "!";
                logAction(logMsg);
                target->loseLives();

                // Trigger upgrade choice when hitting an enemy (not just destroying)
                cout << "\n*** UPGRADE OPPORTUNITY ***" << endl;
                attacker->triggerUpgradeChoice();
                cout << "*** END UPGRADE ***\n"
                     << endl;

                if (!target->isAlive())
                {
                    cout << "[DESTROYED] " << target->getName() << " is destroyed!" << endl;
                }

                if (attacker->getShells() == 0 && shellsBefore > 0)
                {
                    cout << attacker->getName() << " has no shells left!" << endl;
                }
                return true;
            }
            else
            {
                cout << " - [MISS] Shot missed " << target->getName() << "." << endl;
                logMsg += " - MISSED " + target->getName();
            }
        }
        else
        {
            cout << "\n- Shot hits empty space." << endl;
            logMsg += " - Hit empty space";
        }
    }
    else
    {
        cout << "\n- Shot goes out of bounds." << endl;
        logMsg += " - Shot went out of bounds";
    }

    logAction(logMsg);

    // Check shell depletion AFTER whole shooting message
    if (attacker->getShells() == 0 && shellsBefore > 0)
    {
        cout << attacker->getName() << " has no shells left!" << endl;
    }

    return false;
}

bool Battlefield::positionOccupied(int r, int c)
{
    for (const auto &robot : robots)
    {
        if (robot->isAlive() && robot->getRow() == r && robot->getCol() == c)
            return true;
    }
    return false;
}

void Battlefield::lookAround(Robot *robot, int dx, int dy)
{
    int centerRow = robot->getRow() + dx;
    int centerCol = robot->getCol() + dy;

    cout << "[LOOK] " << robot->getName() << " looks around (" << centerRow << "," << centerCol << ")\n";

    string logMsg = "LOOK: " + robot->getName() + " looks around (" +
                    to_string(centerRow) + "," + to_string(centerCol) + ")";

    bool foundEnemy = false;
    vector<string> enemiesFound;

    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {

            if (i == 0 && j == 0)
                continue;

            int r = centerRow + i;
            int c = centerCol + j;

            if (r < 0 || r >= rows || c < 0 || c >= cols)
            {
                continue;
            }
            else
            {
                Robot *target = findTargetAt(r, c, robot);
                if (target)
                {
                    cout << " - Enemy found!!! Robot " << target->getName() << " at (" << r << "," << c << ")" << "\n";
                    enemiesFound.push_back(target->getName() + " at (" + to_string(r) + "," + to_string(c) + ")");
                    foundEnemy = true;
                }
            }
        }
    }

    if (!foundEnemy)
    {
        "- No enemies detected.\n";
        logMsg += " - No enemies detected";
    }
    else
    {
        logMsg += " - Found enemies: ";
        for (size_t i = 0; i < enemiesFound.size(); ++i)
        {
            if (i > 0)
                logMsg += ", ";
            logMsg += enemiesFound[i];
        }
    }

    logAction(logMsg);
};

void Battlefield::simulate()
{
    cout << "\nStarting simulation for " << steps << " steps...\n";
    logAction("=== SIMULATION START ===");

    for (int step = 0; step < steps; step++)
    {
        cout << "\n=== Step " << step + 1 << " ===" << endl;
        logAction("--- STEP " + to_string(step + 1) + " ---");

        robots.erase(remove_if(robots.begin(), robots.end(),
                               [](const unique_ptr<Robot> &r)
                               { return !r->isAlive(); }),
                     robots.end());

        int aliveCount = robots.size();
        if (aliveCount == 1)
        {
            cout << "\n"
                 << robots[0]->getName() << " is the last robot standing and wins the game!\n";
            cout << "Number of survivors: 1" << endl;
            logAction("WINNER: " + robots[0]->getName() + " is the last robot standing and wins!");
            logAction("SURVIVOR_COUNT: 1");
            return;
        }

        if (aliveCount == 0)
        {
            cout << "All robots are dead. No winners this time.\n";
            cout << "Number of survivors: 0" << endl;
            logAction("GAME_END: All robots are dead. No winners.");
            logAction("SURVIVOR_COUNT: 0");
            return;
        }

        for (const auto &robot : robots)
        {
            if (!robot->isAlive())
                continue;

            Robot::Action action = robot->chooseAction();
            switch (action)
            {
            case Robot::THINK:
                robot->think();
                break;
            case Robot::MOVE:
                robot->move();
                break;
            case Robot::SEE:
                robot->see();
                break;
            case Robot::SHOOT:
                robot->shoot();
                break;
            }
        }

        handleCollisions();
        updateGrid();
        display();

        // Only pause for user input if there were upgrade opportunities
        cout << "\nPress Enter to continue to next step...\n";
        cin.get();
    }

    cout << "\nSimulation complete.\n";
    logAction("=== SIMULATION COMPLETE ===");

    // Final results
    if (!robots.empty())
    {
        cout << "\nSurviving robots after all steps:\n";
        cout << "Number of survivors: " << robots.size() << endl;
        string survivors = "SURVIVORS: ";
        for (size_t i = 0; i < robots.size(); ++i)
        {
            cout << " - " << robots[i]->getName() << endl;
            if (i > 0)
                survivors += ", ";
            survivors += robots[i]->getName();
        }
        logAction(survivors);
        logAction("SURVIVOR_COUNT: " + to_string(robots.size()));
    }
    else
    {
        cout << "\nAll robots died. No winners\n";
        cout << "Number of survivors: 0" << endl;
        logAction("FINAL_RESULT: All robots died. No winners.");
        logAction("SURVIVOR_COUNT: 0");
    }
}

class ThinkingRobot
{
public:
    virtual void think() = 0;
    virtual ~ThinkingRobot() = default;
};

class MovingRobot
{
public:
    virtual void move() = 0;
    virtual ~MovingRobot() = default;
};

class SeeingRobot
{
protected:
    vector<pair<int, int>> visiblePositions;

public:
    virtual void see() = 0;
    virtual ~SeeingRobot() = default;
};

class ShootingRobot
{
public:
    virtual bool shoot() = 0;
    virtual ~ShootingRobot() = default;
};

// Enhanced ScoutBot functionality
class ScoutBotAbility
{
private:
    int scoutVisionUses = 3;
    vector<pair<int, int>> visiblePositions;

public:
    bool hasUsesLeft() const { return scoutVisionUses > 0; }

    void performScoutVision(Robot *robot, Battlefield *battlefield);

    void displayVisibleRobots(const string &robotName) const
    {
        cout << robotName << " (Scout Vision) sees the following robots:\n";
        for (const auto &pos : visiblePositions)
        {
            cout << " - Robot at (" << pos.second << ", " << pos.first << ")\n";
        }
    }
};

// Enhanced TrackBot functionality
class TrackBotAbility
{
private:
    int trackersLeft = 3;
    vector<Robot *> trackedRobots;

public:
    bool hasTrackersLeft() const { return trackersLeft > 0; }

    bool plantTracker(Robot *tracker, Robot *target)
    {
        if (trackersLeft > 0 && target && target != tracker)
        {
            // Check if already tracking this robot
            for (Robot *r : trackedRobots)
            {
                if (r == target)
                {
                    cout << tracker->getName() << " is already tracking " << target->getName() << ".\n";
                    return false;
                }
            }
            trackedRobots.push_back(target);
            trackersLeft--;
            cout << tracker->getName() << " planted a tracker on " << target->getName() << ".\n";
            logAction("TRACKER_PLANTED: " + tracker->getName() + " planted tracker on " + target->getName());
            return true;
        }
        return false;
    }

    void displayTrackedLocations(const string &robotName) const
    {
        cout << robotName << " (Tracker) is tracking:\n";
        for (Robot *robot : trackedRobots)
        {
            if (robot->isAlive())
            {
                cout << " - " << robot->getName() << " at (" << robot->getRow() << ", " << robot->getCol() << ")\n";
            }
            else
            {
                cout << " - " << robot->getName() << " [DESTROYED]\n";
            }
        }
    }

    const vector<Robot *> &getTrackedRobots() const { return trackedRobots; }
};

class GenericRobot : public Robot,
                     public ThinkingRobot,
                     public MovingRobot,
                     public SeeingRobot,
                     public ShootingRobot
{
protected:
    static int rows;
    static int cols;
    static Battlefield *battlefield;

    // Seeing upgrade abilities
    unique_ptr<ScoutBotAbility> scoutAbility;
    unique_ptr<TrackBotAbility> trackAbility;

    enum SeeingUpgrade
    {
        NONE,
        SCOUT_BOT,
        TRACK_BOT
    } seeingUpgrade;

    enum ShootingUpgrade
    {
        NONE_SHOOT,
        LONG_SHOT,
        SEMI_AUTO,
        THIRTY_SHOT
    } shootingUpgrade = NONE_SHOOT;

public:
    GenericRobot(const string &name, int r, int c);
    static void setBattlefieldSize(int r, int c);
    static void setBattlefield(Battlefield *bf);

    void think() override;
    void move() override;
    void see() override;
    bool shoot() override;

    // Upgrade methods
    void upgradeTo(SeeingUpgrade upgrade);
    void chooseSeeingUpgrade();
    void chooseShootingUpgrade(); // picks LONG_SHOT / SEMI_AUTO / THIRTY_SHOT at random
    bool longRangeShoot(int range); // your Manhattan-distance ≤ range shot
    bool semiAutoShoot();  

    // Access to abilities for battlefield interactions
    TrackBotAbility *getTrackAbility() { return trackAbility.get(); }

    // Method to move away from current position
    void moveAway();
};

bool Battlefield::readFile(const string &filename)
{
    ifstream file(filename);
    if (!file.is_open())
    {
        cout << "Error opening file!" << endl;
        return false;
    }

    vector<string> lines;
    string line;
    while (getline(file, line))
    {
        lines.push_back(line);
    }
    file.close();

    for (const auto &line : lines)
    {
        if (line.find("M by N") != string::npos)
        {
            stringstream ss(line);
            string temp;
            ss >> temp >> temp >> temp >> temp >> rows >> cols;
            grid.resize(rows, vector<char>(cols, '.'));
            GenericRobot::setBattlefieldSize(rows, cols);
            GenericRobot::setBattlefield(this);
            cout << "Grid size: " << rows << "x" << cols << endl;
            logAction("SETUP: Battlefield initialized with size " + to_string(rows) + "x" + to_string(cols));
        }
        else if (line.find("steps:") != string::npos)
        {
            stringstream ss(line);
            string temp;
            ss >> temp >> steps;
            cout << "Simulation steps: " << steps << endl;
            logAction("SETUP: Simulation will run for " + to_string(steps) + " steps");
        }
        else if (line.find("GenericRobot") != string::npos)
        {
            stringstream ss(line);
            string type, name, rowStr, colStr;
            ss >> type >> name >> rowStr >> colStr;

            int r = (rowStr == "random") ? rand() % rows : stoi(rowStr);
            int c = (colStr == "random") ? rand() % cols : stoi(colStr);

            r = max(0, min(r, rows - 1));
            c = max(0, min(c, cols - 1));

            auto robot = make_unique<GenericRobot>(name, r, c);
            robots.push_back(move(robot));

            cout << "Created " << type << " " << name
                 << " at (" << r << "," << c << ")" << endl;
            logAction("SPAWN: " + name + " created at position (" + to_string(r) + "," + to_string(c) +
                      ") with 3 lives and 10 shells");
        }
    }

    cout << "Total robots created: " << robots.size() << endl;
    logAction("SETUP: Total of " + to_string(robots.size()) + " robots created");
    updateGrid();
    return true;
}

void Battlefield::handleCollisions()
{
    // Create a map to track positions and robots at those positions
    vector<vector<vector<Robot *>>> positionMap(rows, vector<vector<Robot *>>(cols));

    // Fill the position map
    for (const auto &robot : robots)
    {
        if (robot->isAlive())
        {
            int r = robot->getRow();
            int c = robot->getCol();
            positionMap[r][c].push_back(robot.get());
        }
    }

    // Check for collisions (multiple robots at same position)
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            if (positionMap[i][j].size() > 1)
            {
                cout << "\n[COLLISION] Multiple robots at position (" << i << "," << j << ")!" << endl;
                string logMsg = "COLLISION: Robots at position (" + to_string(i) + "," + to_string(j) + "): ";

                for (size_t k = 0; k < positionMap[i][j].size(); ++k)
                {
                    if (k > 0)
                        logMsg += ", ";
                    logMsg += positionMap[i][j][k]->getName();
                    cout << " - " << positionMap[i][j][k]->getName() << endl;
                }
                logAction(logMsg);

                // Make all robots at this position shoot at each other
                cout << "Robots engage in close combat!" << endl;
                cout << "---------------------------------" << endl;
                logAction("CLOSE_COMBAT: Robots engage in automatic close combat");

                // Store robots that survive combat for moving away
                vector<Robot *> survivingRobots;

                for (size_t k = 0; k < positionMap[i][j].size(); ++k)
                {
                    Robot *attacker = positionMap[i][j][k];
                    if (!attacker->isAlive())
                        continue;

                    // Find a target from the same position (excluding self)
                    for (size_t t = 0; t < positionMap[i][j].size(); ++t)
                    {
                        if (t == k)
                            continue; // Skip self

                        Robot *target = positionMap[i][j][t];
                        if (!target->isAlive())
                            continue;

                        // Perform close combat
                        int shellsBefore = attacker->getShells();
                        attacker->useShell();

                        cout << "[CLOSE COMBAT] " << attacker->getName() << " shoots " << target->getName();
                        string combatLog = "CLOSE_COMBAT_SHOOT: " + attacker->getName() + " shoots " + target->getName();

                        // Higher hit chance in close combat (80%)
                        if (rand() % 10 < 8)
                        {
                            cout << " - [HIT] " << target->getName() << " is hit in close combat!" << endl;
                            combatLog += " - HIT in close combat!";
                            logAction(combatLog);
                            target->loseLives();

                            // Trigger upgrade on hit (not just destruction)
                            cout << "\n*** UPGRADE OPPORTUNITY (CLOSE COMBAT) ***" << endl;
                            attacker->triggerUpgradeChoice(); // <-- Add this line
                            cout << "*** END UPGRADE ***\n"
                                 << endl;

                            if (!target->isAlive())
                            {
                                cout << "[DESTROYED] " << target->getName() << " is destroyed in close combat!" << endl;
                            }
                        }
                        else
                        {
                            cout << " - [MISS] Somehow missed in close combat!" << endl;
                            combatLog += " - MISSED in close combat";
                            logAction(combatLog);
                        }

                        if (attacker->getShells() == 0 && shellsBefore > 0)
                        {
                            cout << attacker->getName() << " has no shells left!" << endl;
                        }

                        break; // Each robot shoots once per collision
                    }
                }

                // After combat, make surviving robots move away
                cout << "\n[POST COMBAT] Surviving robots move away from the combat zone..." << endl;
                logAction("POST_COMBAT: Surviving robots attempt to move away from combat zone");

                for (size_t k = 0; k < positionMap[i][j].size(); ++k)
                {
                    Robot *robot = positionMap[i][j][k];
                    if (robot->isAlive())
                    {
                        // Try to cast to GenericRobot to call moveAway
                        GenericRobot *genericRobot = dynamic_cast<GenericRobot *>(robot);
                        if (genericRobot)
                        {
                            genericRobot->moveAway();
                        }
                    }
                }
            }
        }
    }
}

Robot::Robot(const string &robotName, int r, int c)
    : name(robotName), row(r), col(c), symbol(name[0]), lives(3), shells(10)
{
    static bool seeded = false;
    if (!seeded)
    {
        seeded = true;
    }
}

Robot::~Robot() = default;

string Robot::getName() const { return name; }
int Robot::getRow() const { return row; }
int Robot::getCol() const { return col; }
char Robot::getSymbol() const { return symbol; }
int Robot::getLives() const { return lives; }
int Robot::getShells() const { return shells; }

void Robot::setPosition(int r, int c)
{
    row = r;
    col = c;
}

void Robot::destroy()
{
    lives = 0;
    shells = 0;
    logAction("DESTROYED: " + name + " has been destroyed!");
}

void Robot::loseLives()
{
    if (lives > 0)
    {
        lives--;
        logAction("DAMAGE: " + name + " lost a life (Lives remaining: " + to_string(lives) + ")");
        if (lives == 0)
        {
            destroy();
        }
    }
}

bool Robot::isAlive() const
{
    return lives > 0 && shells > 0;
}

void Robot::useShell()
{
    if (shells > 0)
    {
        shells--;
        if (shells == 0)
        {
            logAction("OUT_OF_AMMO: " + name + " has no shells left!");
            destroy();
        }
    }
}

Robot::Action Robot::chooseAction()
{
    return static_cast<Action>(rand() % 4);
}

void Robot::performAction()
{
    if (!isAlive())
        return;
    switch (chooseAction())
    {
    case THINK:
        think();
        break;
    case MOVE:
        move();
        break;
    case SEE:
        see();
        break;
    case SHOOT:
        shoot();
        break;
    }
}

bool Robot::canUpgrade(UpgradeArea area) const
{
    if (chosenUpgrades.size() >= 3)
        return false;
    for (UpgradeArea chosen : chosenUpgrades)
    {
        if (chosen == area)
            return false;
    }
    return true;
}

void Robot::applyUpgrade(UpgradeArea area)
{
    if (!canUpgrade(area))
    {
        cout << name << " cannot upgrade " << getAreaName(area) << " again." << endl;
        return;
    }
    chosenUpgrades.push_back(area);
    cout << name << " upgraded " << getAreaName(area) << "." << endl;
    logAction("UPGRADE: " + name + " upgraded " + getAreaName(area));

    // Dispatch into area‐specific chooser
    if (GenericRobot *g = dynamic_cast<GenericRobot *>(this))
    {
        if (area == SEEING)
        
            g->chooseSeeingUpgrade(); // existing :contentReference[oaicite:0]{index=0}
        else if (area == SHOOTING)
            g->chooseShootingUpgrade();
        // MOVING left intact for your HideBot/JumpBot...
    }
    else
    {
        cout << name << " cannot upgrade " << getAreaName(area) << " again." << endl;
    }
}

string Robot::getAreaName(UpgradeArea area)
{
    switch (area)
    {
    case SEEING:
        return "Seeing";
    case MOVING:
        return "Moving";
    case SHOOTING:
        return "Shooting";
    default:
        return "Unknown";
    }
}

void Robot::triggerUpgradeChoice()
{
    if (chosenUpgrades.size() < 3)
    {
        cout << "\n"
             << name << " hit an enemy and can choose an upgrade!" << endl;
        cout << "Available upgrade areas:" << endl;

        vector<UpgradeArea> availableAreas;
        int optionNumber = 1;

        if (canUpgrade(MOVING))
        {
            cout << optionNumber << ". Moving (HideBot or JumpBot)" << endl;
            availableAreas.push_back(MOVING);
            optionNumber++;
        }
        if (canUpgrade(SHOOTING))
        {
            cout << optionNumber << ". Shooting (LongShotBot, SemiAutoBot or ThirtyShotBot)" << endl;
            availableAreas.push_back(SHOOTING);
            optionNumber++;
        }
        if (canUpgrade(SEEING))
        {
            cout << optionNumber << ". Seeing (ScoutBot or TrackBot)" << endl;
            availableAreas.push_back(SEEING);
            optionNumber++;
        }

        if (!availableAreas.empty())
        {
            cout << "Enter your choice (1-" << availableAreas.size() << "): ";
            int choice;
            cin >> choice;

            // Validate input
            if (choice >= 1 && choice <= (int)availableAreas.size())
            {
                UpgradeArea chosenArea = availableAreas[choice - 1];
                cout << name << " chooses to upgrade " << getAreaName(chosenArea) << "!" << endl;
                applyUpgrade(chosenArea);
            }
            else
            {
                cout << "Invalid choice. No upgrade applied." << endl;
            }
        }
    }
    else
    {
        cout << name << " has already reached maximum upgrades (3/3)." << endl;
    }
}

// ScoutBotAbility Implementation
void ScoutBotAbility::performScoutVision(Robot *robot, Battlefield *battlefield)
{
    if (scoutVisionUses > 0)
    {
        visiblePositions.clear();
        vector<Robot *> allRobots = battlefield->getAllAliveRobots();

        for (Robot *otherRobot : allRobots)
        {
            if (otherRobot != robot)
            {
                visiblePositions.push_back({otherRobot->getCol(), otherRobot->getRow()});
            }
        }

        scoutVisionUses--;
        displayVisibleRobots(robot->getName());
        logAction("SCOUT_VISION: " + robot->getName() + " used scout vision (" +
                  to_string(scoutVisionUses) + " uses remaining)");
    }
    else
    {
        cout << robot->getName() << " has no scout vision uses left." << endl;
    }
}

int GenericRobot::rows = 0;
int GenericRobot::cols = 0;
Battlefield *GenericRobot::battlefield = nullptr;

GenericRobot::GenericRobot(const string &name, int r, int c)
    : Robot(name, r, c), seeingUpgrade(NONE) {}

void GenericRobot::setBattlefieldSize(int r, int c)
{
    rows = r;
    cols = c;
}

void GenericRobot::setBattlefield(Battlefield *bf)
{
    battlefield = bf;
}

void GenericRobot::think()
{
    cout << name << " is thinking strategically." << endl;
    logAction("THINK: " + name + " is thinking strategically at position (" + to_string(row) + "," + to_string(col) + ")");
}

void GenericRobot::move()
{
    int oldRow = row, oldCol = col;
    int direction = rand() % 8;
    switch (direction)
    {
    case 0:
        row = max(0, row - 1);
        break;
    case 1:
        row = min(rows - 1, row + 1);
        break;
    case 2:
        col = max(0, col - 1);
        break;
    case 3:
        col = min(cols - 1, col + 1);
        break;
    case 4:
        row = max(0, row - 1);
        col = max(0, col - 1);
        break;
    case 5:
        row = max(0, row - 1);
        col = min(cols - 1, col + 1);
        break;
    case 6:
        row = min(rows - 1, row + 1);
        col = max(0, col - 1);
        break;
    case 7:
        row = min(rows - 1, row + 1);
        col = min(cols - 1, col + 1);
        break;
    }
    cout << "[MOVE] " << name << " moved to (" << row << "," << col << ")" << endl;
    logAction("MOVE: " + name + " moved from (" + to_string(oldRow) + "," + to_string(oldCol) +
              ") to (" + to_string(row) + "," + to_string(col) + ") - Direction: " + to_string(direction));
}

void GenericRobot::moveAway()
{
    int oldRow = row, oldCol = col;

    // Try to find an empty adjacent position to move to
    int directions[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    vector<int> validDirections;

    // Check which directions are valid and don't have other robots
    for (int i = 0; i < 8; i++)
    {
        int newRow = row + directions[i][0];
        int newCol = col + directions[i][1];

        // Check bounds
        if (newRow >= 0 && newRow < rows && newCol >= 0 && newCol < cols)
        {
            // Check if position is empty (this will be checked by battlefield)
            validDirections.push_back(i);
        }
    }

    if (!validDirections.empty())
    {
        // Pick a random valid direction
        int dirIndex = validDirections[rand() % validDirections.size()];
        int newRow = row + directions[dirIndex][0];
        int newCol = col + directions[dirIndex][1];

        row = newRow;
        col = newCol;

        cout << "[MOVE AWAY] " << name << " moves away from combat to (" << row << "," << col << ")" << endl;
        logAction("MOVE_AWAY: " + name + " moved away from combat from (" + to_string(oldRow) + "," +
                  to_string(oldCol) + ") to (" + to_string(row) + "," + to_string(col) + ")");
    }
    else
    {
        cout << "[MOVE AWAY] " << name << " has nowhere to move away!" << endl;
        logAction("MOVE_AWAY: " + name + " has nowhere to move away from (" + to_string(row) + "," + to_string(col) + ")");
    }
}

void GenericRobot::see()
{
    if (!battlefield)
        return;

    switch (seeingUpgrade)
    {
    case SCOUT_BOT:
        if (scoutAbility)
        {
            scoutAbility->performScoutVision(this, battlefield);
        }
        else
        {
            battlefield->lookAround(this, 0, 0);
        }
        break;

    case TRACK_BOT:
        if (trackAbility)
        {
            trackAbility->displayTrackedLocations(name);

            // Try to plant a tracker on a nearby robot
            if (trackAbility->hasTrackersLeft())
            {
                Robot *nearestRobot = battlefield->findNearestRobot(this);
                if (nearestRobot)
                {
                    // Only plant tracker if robot is close enough (within 2 squares)
                    int distance = abs(nearestRobot->getRow() - row) + abs(nearestRobot->getCol() - col);
                    if (distance <= 2)
                    {
                        trackAbility->plantTracker(this, nearestRobot);
                    }
                }
            }
        }
        else
        {
            battlefield->lookAround(this, 0, 0);
        }
        break;

    default:
        battlefield->lookAround(this, 0, 0);
        break;
    }
}

bool GenericRobot::shoot()
{
    if (!battlefield)
        return false;

    switch (shootingUpgrade)
    {
    case LONG_SHOT:
        return longRangeShoot(3);
    case SEMI_AUTO:
        return semiAutoShoot();
    case THIRTY_SHOT:
        // after reload, fall back to normal shooting
        return battlefield->handleCombat(this);
    default:
        return battlefield->handleCombat(this);
    }
}

void GenericRobot::chooseShootingUpgrade()
{
    static const vector<ShootingUpgrade> options = {
        LONG_SHOT, SEMI_AUTO, THIRTY_SHOT};
    // pick one at random
    shootingUpgrade = options[rand() % options.size()];

    switch (shootingUpgrade)
    {
    case LONG_SHOT:
        cout << name << " upgraded to LongShotBot! (can shoot up to distance 3)" << endl;
        logAction("SHOOTING_UPGRADE: " + name + " → LongShotBot");
        break;
    case SEMI_AUTO:
        cout << name << " upgraded to SemiAutoBot! (3 shots per shell, 70% each)" << endl;
        logAction("SHOOTING_UPGRADE: " + name + " → SemiAutoBot");
        break;
    case THIRTY_SHOT:
        shells = 30; // clear and reload
        cout << name << " upgraded to ThirtyShotBot! (shells now 30)" << endl;
        logAction("SHOOTING_UPGRADE: " + name + " → ThirtyShotBot (shells=30)");
        break;
    default:
        break;
    }
}

// Shoot to any target within manhattan-distance <= range.
bool GenericRobot::longRangeShoot(int range)
{
    // consume one shell
    this->useShell();

    // pick a random (dx,dy) with |dx|+|dy| <= range and not (0,0)
    vector<pair<int, int>> candidates;
    for (int dx = -range; dx <= range; ++dx)
        for (int dy = -range; dy <= range; ++dy)
            if ((abs(dx) + abs(dy) > 0) && (abs(dx) + abs(dy) <= range))
                candidates.emplace_back(dx, dy);

    auto [dx, dy] = candidates[rand() % candidates.size()];
    int tr = row + dx, tc = col + dy;
    cout << name << " (LongShot) fires to (" << tr << "," << tc << ")";

    if (Robot *target = battlefield->findTargetAt(tr, tc, this))
    {
        // 70% hit chance
        if (rand() % 10 < 7)
        {
            cout << " - [HIT] " << target->getName() << "!\n";
            target->loseLives();
            logAction("LONG_SHOT HIT: " + name);
            triggerUpgradeChoice();
            return true;
        }
        else
        {
            cout << " - [MISS]\n";
            logAction("LONG_SHOT MISS: " + name);
        }
    }
    else
    {
        cout << " - no target.\n";
        logAction("LONG_SHOT EMPTY: " + name);
    }
    return false;
}

// Consume one shell, but fire three 70%‐chance shots at the same adjacent cell.
bool GenericRobot::semiAutoShoot()
{
    this->useShell();

    // choose one adjacent direction (dx,dy) as in base handleCombat...
    int dir = rand() % 8;
    static int DX[8] = {-1, -1, -1, 0, 0, 1, 1, 1}, DY[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int tr = row + DX[dir], tc = col + DY[dir];
    cout << name << " (SemiAuto) fires 3-round burst to (" << tr << "," << tc << ")\n";

    if (Robot *target = battlefield->findTargetAt(tr, tc, this))
    {
        for (int shot = 1; shot <= 3; ++shot)
        {
            bool hit = (rand() % 100) < 70;
            cout << "  shot " << shot << (hit ? " HIT\n" : " miss\n");
            if (hit)
            {
                target->loseLives(); // or target->destroy();
                logAction("SEMI_AUTO HIT: " + name);
                triggerUpgradeChoice();
                if (!target->isAlive())
                    break;
            }
        }
        return true;
    }
    else
    {
        cout << "  no target at that location.\n";
        logAction("SEMI_AUTO EMPTY: " + name);
        return false;
    }
}

void GenericRobot::chooseSeeingUpgrade()
{
    cout << "\nChoose seeing upgrade for " << name << ":" << endl;
    cout << "1. ScoutBot - Can see all robots on the battlefield (3 uses)" << endl;
    cout << "2. TrackBot - Can plant trackers on robots to track their movements (3 trackers)" << endl;

    // For simulation, randomly choose
    int choice = (rand() % 2) + 1;

    switch (choice)
    {
    case 1:
        upgradeTo(SCOUT_BOT);
        cout << name << " upgraded to ScoutBot!" << endl;
        logAction("SEEING_UPGRADE: " + name + " upgraded to ScoutBot");
        break;
    case 2:
        upgradeTo(TRACK_BOT);
        cout << name << " upgraded to TrackBot!" << endl;
        logAction("SEEING_UPGRADE: " + name + " upgraded to TrackBot");
        break;
    }
}

void GenericRobot::upgradeTo(SeeingUpgrade upgrade)
{
    seeingUpgrade = upgrade;

    switch (upgrade)
    {
    case SCOUT_BOT:
        scoutAbility = make_unique<ScoutBotAbility>();
        break;
    case TRACK_BOT:
        trackAbility = make_unique<TrackBotAbility>();
        break;
    default:
        break;
    }
}

int main()
{
    srand(time(nullptr));

    // Create log file with timestamp
    time_t now = time(0);
    char timeStr[100];
    strftime(timeStr, sizeof(timeStr), "%Y%m%d_%H%M%S", localtime(&now));
    string logFilename = "robot_battle_actions_" + string(timeStr) + ".log";

    // Open log file
    logFile.open(logFilename);
    if (!logFile.is_open())
    {
        cerr << "Warning: Could not create log file " << logFilename << endl;
    }
    else
    {
        logAction("===== ROBOT BATTLE ACTION LOG =====");
        logAction("Timestamp: " + string(timeStr));
        logAction("====================================");
    }

    cout << "Robot Battle Simulation Starting..." << endl;
    cout << "Action log will be saved to: " << logFilename << endl;

    Battlefield battlefield;
    if (battlefield.readFile("setup.txt"))
    {
        battlefield.display();
        cout << "\nPress Enter to start simulation...";
        cin.get();
        battlefield.simulate();
    }
    else
    {
        cout << "Failed to initialize battlefield." << endl;
        logAction("ERROR: Failed to initialize battlefield");
        if (logFile.is_open())
        {
            logFile.close();
        }
        return 1;
    }

    if (logFile.is_open())
    {
        logAction("====================================");
        logAction("===== ACTION LOG END =====");
        logFile.close();
    }

    cout << "\nSimulation completed!" << endl;
    cout << "Detailed action log saved to: " << logFilename << endl;

    return 0;
}