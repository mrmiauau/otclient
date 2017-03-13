/*
 * Copyright (c) 2010-2017 OTClient <https://github.com/edubart/otclient>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "map.h"
#include "game.h"
#include "localplayer.h"
#include "tile.h"
#include "item.h"
#include "missile.h"
#include "statictext.h"
#include "mapview.h"
#include "minimap.h"

#include <framework/core/eventdispatcher.h>
#include <framework/core/application.h>
#include <set>

Map g_map;
TilePtr Map::m_nulltile;

void Map::init()
{
    resetAwareRange();
    m_animationFlags |= Animation_Show;
}

void Map::terminate()
{
    clean();
}

void Map::addMapView(const MapViewPtr& mapView)
{
    m_mapViews.push_back(mapView);
}

void Map::removeMapView(const MapViewPtr& mapView)
{
    auto it = std::find(m_mapViews.begin(), m_mapViews.end(), mapView);
    if(it != m_mapViews.end())
        m_mapViews.erase(it);
}

void Map::notificateTileUpdate(const Position& pos)
{
    if(!pos.isMapPosition())
        return;

    for(const MapViewPtr& mapView : m_mapViews)
        mapView->onTileUpdate(pos);
    g_minimap.updateTile(pos, getTile(pos));
}

void Map::clean()
{
    cleanDynamicThings();

    for(int i=0;i<=Otc::MAX_Z;++i)
        m_tileBlocks[i].clear();

    m_waypoints.clear();

    g_towns.clear();
    g_houses.clear();
    g_creatures.clearSpawns();
    m_tilesRect = Rect(65534, 65534, 0, 0);
}

void Map::cleanDynamicThings()
{
    for(const auto& pair : m_knownCreatures) {
        const CreaturePtr& creature = pair.second;
        removeThing(creature);
    }
    m_knownCreatures.clear();

    for(int i=0;i<=Otc::MAX_Z;++i)
        m_floorMissiles[i].clear();

    cleanTexts();
}

void Map::cleanTexts()
{
    m_animatedTexts.clear();
    m_staticTexts.clear();
}

void Map::addThing(const ThingPtr& thing, const Position& pos, int stackPos)
{
    if(!thing)
        return;

    if(thing->isItem() || thing->isCreature() || thing->isEffect()) {
        const TilePtr& tile = getOrCreateTile(pos);
        if(tile)
            tile->addThing(thing, stackPos);
    } else {
        if(thing->isMissile()) {
            m_floorMissiles[pos.z].push_back(thing->static_self_cast<Missile>());
            thing->onAppear();
        } else if(thing->isAnimatedText()) {
            // this code will stack animated texts of the same color
            AnimatedTextPtr animatedText = thing->static_self_cast<AnimatedText>();
            AnimatedTextPtr prevAnimatedText;
            bool merged = false;
            for(auto other : m_animatedTexts) {
                if(other->getPosition() == pos) {
                    prevAnimatedText = other;
                    if(other->merge(animatedText)) {
                        merged = true;
                        break;
                    }
                }
            }
            if(!merged) {
                if(prevAnimatedText) {
                    Point offset = prevAnimatedText->getOffset();
                    float t = prevAnimatedText->getTimer().ticksElapsed();
                    if(t < Otc::ANIMATED_TEXT_DURATION / 4.0) { // didnt move 12 pixels
                        int y = 12 - 48 * t / (float)Otc::ANIMATED_TEXT_DURATION;
                        offset += Point(0, y);
                    }
                    offset.y = std::min<int>(offset.y, 12);
                    animatedText->setOffset(offset);
                }
                m_animatedTexts.push_back(animatedText);
            }
        } else if(thing->isStaticText()) {
            StaticTextPtr staticText = thing->static_self_cast<StaticText>();
            bool mustAdd = true;
            for(auto other : m_staticTexts) {
                // try to combine messages
                if(other->getPosition() == pos && other->addMessage(staticText->getName(), staticText->getMessageMode(), staticText->getFirstMessage())) {
                    mustAdd = false;
                    break;
                }
            }

            if(mustAdd)
                m_staticTexts.push_back(staticText);
            else
                return;
        }

        thing->setPosition(pos);
        thing->onAppear();
    }

    notificateTileUpdate(pos);
}

ThingPtr Map::getThing(const Position& pos, int stackPos)
{
    if(TilePtr tile = getTile(pos))
        return tile->getThing(stackPos);
    return nullptr;
}

bool Map::removeThing(const ThingPtr& thing)
{
    if(!thing)
        return false;

    bool ret = false;
    if(thing->isMissile()) {
        MissilePtr missile = thing->static_self_cast<Missile>();
        int z = missile->getPosition().z;
        auto it = std::find(m_floorMissiles[z].begin(), m_floorMissiles[z].end(), missile);
        if(it != m_floorMissiles[z].end()) {
            m_floorMissiles[z].erase(it);
            ret = true;
        }
    } else if(thing->isAnimatedText()) {
        AnimatedTextPtr animatedText = thing->static_self_cast<AnimatedText>();
        auto it = std::find(m_animatedTexts.begin(), m_animatedTexts.end(), animatedText);
        if(it != m_animatedTexts.end()) {
            m_animatedTexts.erase(it);
            ret = true;
        }
    } else if(thing->isStaticText()) {
        StaticTextPtr staticText = thing->static_self_cast<StaticText>();
        auto it = std::find(m_staticTexts.begin(), m_staticTexts.end(), staticText);
        if(it != m_staticTexts.end()) {
            m_staticTexts.erase(it);
            ret = true;
        }
    } else if(const TilePtr& tile = thing->getTile())
        ret = tile->removeThing(thing);

    notificateTileUpdate(thing->getPosition());
    return ret;
}

bool Map::removeThingByPos(const Position& pos, int stackPos)
{
    if(TilePtr tile = getTile(pos))
        return removeThing(tile->getThing(stackPos));
    return false;
}

void Map::colorizeThing(const ThingPtr& thing, const Color& color)
{
    if(!thing)
        return;

    if(thing->isItem())
        thing->static_self_cast<Item>()->setColor(color);
    else if(thing->isCreature()) {
        const TilePtr& tile = thing->getTile();
        assert(tile);

        const ThingPtr& topThing = tile->getTopThing();
        assert(topThing);

        topThing->static_self_cast<Item>()->setColor(color);
    }
}

void Map::removeThingColor(const ThingPtr& thing)
{
    if(!thing)
        return;

    if(thing->isItem())
        thing->static_self_cast<Item>()->setColor(Color::alpha);
    else if(thing->isCreature()) {
        const TilePtr& tile = thing->getTile();
        assert(tile);

        const ThingPtr& topThing = tile->getTopThing();
        assert(topThing);

        topThing->static_self_cast<Item>()->setColor(Color::alpha);
    }
}

StaticTextPtr Map::getStaticText(const Position& pos)
{
    for(auto staticText : m_staticTexts) {
        // try to combine messages
        if(staticText->getPosition() == pos)
            return staticText;
    }
    return nullptr;
}

const TilePtr& Map::createTile(const Position& pos)
{
    if(!pos.isMapPosition())
        return m_nulltile;
    if(pos.x < m_tilesRect.left())
        m_tilesRect.setLeft(pos.x);
    if(pos.y < m_tilesRect.top())
        m_tilesRect.setTop(pos.y);
    if(pos.x > m_tilesRect.right())
        m_tilesRect.setRight(pos.x);
    if(pos.y > m_tilesRect.bottom())
        m_tilesRect.setBottom(pos.y);
    TileBlock& block = m_tileBlocks[pos.z][getBlockIndex(pos)];
    return block.create(pos);
}

template <typename... Items>
const TilePtr& Map::createTileEx(const Position& pos, const Items&... items)
{
    if(!pos.isValid())
        return m_nulltile;
    const TilePtr& tile = getOrCreateTile(pos);
    auto vec = {items...};
    for(auto it : vec)
        addThing(it, pos);

    return tile;
}

const TilePtr& Map::getOrCreateTile(const Position& pos)
{
    if(!pos.isMapPosition())
        return m_nulltile;
    if(pos.x < m_tilesRect.left())
        m_tilesRect.setLeft(pos.x);
    if(pos.y < m_tilesRect.top())
        m_tilesRect.setTop(pos.y);
    if(pos.x > m_tilesRect.right())
        m_tilesRect.setRight(pos.x);
    if(pos.y > m_tilesRect.bottom())
        m_tilesRect.setBottom(pos.y);
    TileBlock& block = m_tileBlocks[pos.z][getBlockIndex(pos)];
    return block.getOrCreate(pos);
}

const TilePtr& Map::getTile(const Position& pos)
{
    if(!pos.isMapPosition())
        return m_nulltile;
    auto it = m_tileBlocks[pos.z].find(getBlockIndex(pos));
    if(it != m_tileBlocks[pos.z].end())
        return it->second.get(pos);
    return m_nulltile;
}

const TileList Map::getTiles(int floor/* = -1*/)
{
    TileList tiles;
    if(floor > Otc::MAX_Z) {
        return tiles;
    }
    else if(floor < 0) {
        // Search all floors
        for(uint8_t z = 0; z <= Otc::MAX_Z; ++z) {
            for(const auto& pair : m_tileBlocks[z]) {
                const TileBlock& block = pair.second;
                for(const TilePtr& tile : block.getTiles()) {
                    if(tile != nullptr)
                        tiles.push_back(tile);
                }
            }
        }
    }
    else {
        for(const auto& pair : m_tileBlocks[floor]) {
            const TileBlock& block = pair.second;
            for(const TilePtr& tile : block.getTiles()) {
                if(tile != nullptr)
                    tiles.push_back(tile);
            }
        }
    }
    return tiles;
}

void Map::cleanTile(const Position& pos)
{
    if(!pos.isMapPosition())
        return;
    auto it = m_tileBlocks[pos.z].find(getBlockIndex(pos));
    if(it != m_tileBlocks[pos.z].end()) {
        TileBlock& block = it->second;
        if(const TilePtr& tile = block.get(pos)) {
            tile->clean();
            if(tile->canErase())
                block.remove(pos);

            notificateTileUpdate(pos);
        }
    }
    for(auto it = m_staticTexts.begin();it != m_staticTexts.end();) {
        const StaticTextPtr& staticText = *it;
        if(staticText->getPosition() == pos && staticText->getMessageMode() == Otc::MessageNone)
            it = m_staticTexts.erase(it);
        else
            ++it;
    }
}

void Map::setShowZone(tileflags_t zone, bool show)
{
    if(show)
        m_zoneFlags |= (uint32)zone;
    else
        m_zoneFlags &= ~(uint32)zone;
}

void Map::setShowZones(bool show)
{
    if(!show)
        m_zoneFlags = 0;
    else if(m_zoneFlags == 0)
        m_zoneFlags = TILESTATE_HOUSE | TILESTATE_PROTECTIONZONE;
}

void Map::setZoneColor(tileflags_t zone, const Color& color)
{
    if((m_zoneFlags & zone) == zone)
        m_zoneColors[zone] = color;
}

Color Map::getZoneColor(tileflags_t flag)
{
    auto it = m_zoneColors.find(flag);
    if(it == m_zoneColors.end())
        return Color::alpha;
    return it->second;
}

void Map::setForceShowAnimations(bool force)
{
    if(force) {
        if(!(m_animationFlags & Animation_Force))
            m_animationFlags |= Animation_Force;
    } else
        m_animationFlags &= ~Animation_Force;
}

bool Map::isForcingAnimations()
{
    return (m_animationFlags & Animation_Force) == Animation_Force;
}

bool Map::isShowingAnimations()
{
    return (m_animationFlags & Animation_Show) == Animation_Show;
}

void Map::setShowAnimations(bool show)
{
    if(show) {
        if(!(m_animationFlags & Animation_Show))
            m_animationFlags |= Animation_Show;
    } else
        m_animationFlags &= ~Animation_Show;
}

void Map::beginGhostMode(float opacity)
{
    g_painter->setOpacity(opacity);
}

void Map::endGhostMode()
{
    g_painter->resetOpacity();
}

std::map<Position, ItemPtr> Map::findItemsById(uint16 clientId, uint32 max)
{
    std::map<Position, ItemPtr> ret;
    uint32 count = 0;
    for(uint8_t z = 0; z <= Otc::MAX_Z; ++z) {
        for(const auto& pair : m_tileBlocks[z]) {
            const TileBlock& block = pair.second;
            for(const TilePtr& tile : block.getTiles()) {
                if(unlikely(!tile || tile->isEmpty()))
                    continue;
                for(const ItemPtr& item : tile->getItems()) {
                    if(item->getId() == clientId) {
                        ret.insert(std::make_pair(tile->getPosition(), item));
                        if(++count >= max)
                            break;
                    }
                }
            }
        }
    }

    return ret;
}

void Map::addCreature(const CreaturePtr& creature)
{
    m_knownCreatures[creature->getId()] = creature;
}

CreaturePtr Map::getCreatureById(uint32 id)
{
    auto it = m_knownCreatures.find(id);
    if(it == m_knownCreatures.end())
        return nullptr;
    return it->second;
}

void Map::removeCreatureById(uint32 id)
{
    if(id == 0)
        return;

    auto it = m_knownCreatures.find(id);
    if(it != m_knownCreatures.end())
        m_knownCreatures.erase(it);
}

void Map::removeUnawareThings()
{
    // remove creatures from tiles that we are not aware of anymore
    for(const auto& pair : m_knownCreatures) {
        const CreaturePtr& creature = pair.second;
        if(!isAwareOfPosition(creature->getPosition()))
            removeThing(creature);
    }

    // remove static texts from tiles that we are not aware anymore
    for(auto it = m_staticTexts.begin(); it != m_staticTexts.end();) {
        const StaticTextPtr& staticText = *it;
        if(staticText->getMessageMode() == Otc::MessageNone && !isAwareOfPosition(staticText->getPosition()))
            it = m_staticTexts.erase(it);
        else
            ++it;
    }

    if(!g_game.getFeature(Otc::GameKeepUnawareTiles)) {
        // remove tiles that we are not aware anymore
        for(int z = 0; z <= Otc::MAX_Z; ++z) {
            std::unordered_map<uint, TileBlock>& tileBlocks = m_tileBlocks[z];
            for(auto it = tileBlocks.begin(); it != tileBlocks.end();) {
                TileBlock& block = (*it).second;
                bool blockEmpty = true;
                for(const TilePtr& tile : block.getTiles()) {
                    if(!tile)
                        continue;

                    const Position& pos = tile->getPosition();
                    if(!isAwareOfPosition(pos))
                        block.remove(pos);
                    else
                        blockEmpty = false;
                }

                if(blockEmpty)
                    it = tileBlocks.erase(it);
                else
                    ++it;
            }
        }
    }
}

void Map::setCentralPosition(const Position& centralPosition)
{
    if(m_centralPosition == centralPosition)
        return;

    m_centralPosition = centralPosition;

    removeUnawareThings();

    // this fixes local player position when the local player is removed from the map,
    // the local player is removed from the map when there are too many creatures on his tile,
    // so there is no enough stackpos to the server send him
    g_dispatcher.addEvent([this] {
        LocalPlayerPtr localPlayer = g_game.getLocalPlayer();
        if(!localPlayer || localPlayer->getPosition() == m_centralPosition)
            return;
        TilePtr tile = localPlayer->getTile();
        if(tile && tile->hasThing(localPlayer))
            return;

        Position oldPos = localPlayer->getPosition();
        Position pos = m_centralPosition;
        if(oldPos != pos) {
            if(!localPlayer->isRemoved())
                localPlayer->onDisappear();
            localPlayer->setPosition(pos);
            localPlayer->onAppear();
            g_logger.debug("forced player position update");
        }
    });

    for(const MapViewPtr& mapView : m_mapViews)
        mapView->onMapCenterChange(centralPosition);
}

std::vector<CreaturePtr> Map::getSightSpectators(const Position& centerPos, bool multiFloor)
{
    return getSpectatorsInRangeEx(centerPos, multiFloor, m_awareRange.left - 1, m_awareRange.right - 2, m_awareRange.top - 1, m_awareRange.bottom - 2);
}

std::vector<CreaturePtr> Map::getSpectators(const Position& centerPos, bool multiFloor)
{
    return getSpectatorsInRangeEx(centerPos, multiFloor, m_awareRange.left, m_awareRange.right, m_awareRange.top, m_awareRange.bottom);
}

std::vector<CreaturePtr> Map::getSpectatorsInRange(const Position& centerPos, bool multiFloor, int xRange, int yRange)
{
    return getSpectatorsInRangeEx(centerPos, multiFloor, xRange, xRange, yRange, yRange);
}

std::vector<CreaturePtr> Map::getSpectatorsInRangeEx(const Position& centerPos, bool multiFloor, int minXRange, int maxXRange, int minYRange, int maxYRange)
{
    int minZRange = 0;
    int maxZRange = 0;
    std::vector<CreaturePtr> creatures;

    if(multiFloor) {
        minZRange = 0;
        maxZRange = Otc::MAX_Z;
    }

    //TODO: optimize
    //TODO: get creatures from other floors corretly
    //TODO: delivery creatures in distance order

    for(int iz=-minZRange; iz<=maxZRange; ++iz) {
        for(int iy=-minYRange; iy<=maxYRange; ++iy) {
            for(int ix=-minXRange; ix<=maxXRange; ++ix) {
                TilePtr tile = getTile(centerPos.translated(ix,iy,iz));
                if(!tile)
                    continue;

                auto tileCreatures = tile->getCreatures();
                creatures.insert(creatures.end(), tileCreatures.rbegin(), tileCreatures.rend());
            }
        }
    }

    return creatures;
}

bool Map::isLookPossible(const Position& pos)
{
    TilePtr tile = getTile(pos);
    return tile && tile->isLookPossible();
}

bool Map::isCovered(const Position& pos, int firstFloor)
{
    // check for tiles on top of the postion
    Position tilePos = pos;
    while(tilePos.coveredUp() && tilePos.z >= firstFloor) {
        TilePtr tile = getTile(tilePos);
        // the below tile is covered when the above tile has a full ground
        if(tile && tile->isFullGround())
            return true;
    }
    return false;
}

bool Map::isCompletelyCovered(const Position& pos, int firstFloor)
{
    const TilePtr& checkTile = getTile(pos);
    Position tilePos = pos;
    while(tilePos.coveredUp() && tilePos.z >= firstFloor) {
        bool covered = true;
        bool done = false;
        // check in 2x2 range tiles that has no transparent pixels
        for(int x=0;x<2 && !done;++x) {
            for(int y=0;y<2 && !done;++y) {
                const TilePtr& tile = getTile(tilePos.translated(-x, -y));
                if(!tile || !tile->isFullyOpaque()) {
                    covered = false;
                    done = true;
                } else if(x==0 && y==0 && (!checkTile || checkTile->isSingleDimension())) {
                    done = true;
                }
            }
        }
        if(covered)
            return true;
    }
    return false;
}

bool Map::isAwareOfPosition(const Position& pos)
{
    if(pos.z < getFirstAwareFloor() || pos.z > getLastAwareFloor())
        return false;

    Position groundedPos = pos;
    while(groundedPos.z != m_centralPosition.z) {
        if(groundedPos.z > m_centralPosition.z) {
            if(groundedPos.x == 65535 || groundedPos.y == 65535) // When pos == 65535,65535,15 we cant go up to 65536,65536,14
                break;
            groundedPos.coveredUp();
        }
        else {
            if(groundedPos.x == 0 || groundedPos.y == 0) // When pos == 0,0,0 we cant go down to -1,-1,1
                break;
            groundedPos.coveredDown();
        }
    }
    return m_centralPosition.isInRange(groundedPos, m_awareRange.left,
                                                    m_awareRange.right,
                                                    m_awareRange.top,
                                                    m_awareRange.bottom);
}

void Map::setAwareRange(const AwareRange& range)
{
    m_awareRange = range;
    removeUnawareThings();
}

void Map::resetAwareRange()
{
    AwareRange range;
    range.left = 8;
    range.top = 6;
    range.bottom = 7;
    range.right = 9;
    setAwareRange(range);
}

int Map::getFirstAwareFloor()
{
    if(m_centralPosition.z > Otc::SEA_FLOOR)
        return m_centralPosition.z-Otc::AWARE_UNDEGROUND_FLOOR_RANGE;
    else
        return 0;
}

int Map::getLastAwareFloor()
{
    if(m_centralPosition.z > Otc::SEA_FLOOR)
        return std::min<int>(m_centralPosition.z+Otc::AWARE_UNDEGROUND_FLOOR_RANGE, (int)Otc::MAX_Z);
    else
        return Otc::SEA_FLOOR;
}

std::tuple<std::vector<Otc::Direction>, Otc::PathFindResult> Map::findPath(const Position& startPos, const Position& goalPos, int maxComplexity, int flags)
{
    // pathfinding using A* search algorithm
    // as described in http://en.wikipedia.org/wiki/A*_search_algorithm

    struct Node {
        Node(const Position& pos) : cost(0), totalCost(0), pos(pos), prev(nullptr), dir(Otc::InvalidDirection) { }
        float cost;
        float totalCost;
        Position pos;
        Node *prev;
        Otc::Direction dir;
    };

    struct LessNode : std::binary_function<std::pair<Node*, float>, std::pair<Node*, float>, bool> {
        bool operator()(std::pair<Node*, float> a, std::pair<Node*, float> b) const {
            return b.second < a.second;
        }
    };

    std::tuple<std::vector<Otc::Direction>, Otc::PathFindResult> ret;
    std::vector<Otc::Direction>& dirs = std::get<0>(ret);
    Otc::PathFindResult& result = std::get<1>(ret);

    result = Otc::PathFindResultNoWay;

    if(startPos == goalPos) {
        result = Otc::PathFindResultSamePosition;
        return ret;
    }

    if(!(flags & Otc::PathFindMultiFloor) && (startPos.z != goalPos.z)) {
        result = Otc::PathFindResultImpossible;
        return ret;
    }

    // check the goal pos is walkable
    if(g_map.isAwareOfPosition(goalPos)) {
        const TilePtr goalTile = getTile(goalPos);
        if(!goalTile || !goalTile->isWalkable(flags & Otc::PathFindAllowCreatures)) {
            return ret;
        }
    }
    else {
        const MinimapTile& goalTile = g_minimap.getTile(goalPos);
        if(goalTile.hasFlag(MinimapTileNotWalkable)) {
            return ret;
        }
    }

    std::unordered_map<Position, Node*, PositionHasher> nodes;
    std::priority_queue<std::pair<Node*, float>, std::vector<std::pair<Node*, float>>, LessNode> searchList;

    Node *currentNode = new Node(startPos);
    currentNode->pos = startPos;
    nodes[startPos] = currentNode;
    Node *foundNode = nullptr;
    while(currentNode) {
        if((int)nodes.size() > maxComplexity) {
            result = Otc::PathFindResultTooFar;
            break;
        }

        // path found
        if(currentNode->pos == goalPos && (!foundNode || currentNode->cost < foundNode->cost))
            foundNode = currentNode;

        // cost too high
        if(foundNode && currentNode->totalCost >= foundNode->cost)
            break;

        for(int i=-1;i<=1;++i) {
            for(int j=-1;j<=1;++j) {
                Position neighborPos = currentNode->pos.translated(i, j);
                Otc::Direction walkDir = currentNode->pos.getDirectionFromPosition(neighborPos);
                if(i == 0 && j == 0) {
                    if (!(flags & Otc::PathFindMultiFloor))
                        continue;

                    int floorChange = Otc::FloorChangeNone;

                    if(g_map.isAwareOfPosition(currentNode->pos)) {
                        if(const TilePtr& tile = getTile(currentNode->pos)) {
                            floorChange = tile->getFloorChange();
                        }
                    } else {
                        const MinimapTile& mtile = g_minimap.getTile(neighborPos);
                        floorChange = mtile.getFloorChange();
                    }
                    
                    if (floorChange == Otc::FloorChangeNone)
                        continue;

                    walkDir = Otc::InvalidDirection;

                    if (floorChange & Otc::FloorChangeUp)
                        neighborPos.up(1);

                    if (floorChange & Otc::FloorChangeDown)
                        neighborPos.down(1);
                }

                bool wasSeen = false;
                bool hasCreature = false;
                bool isNotWalkable = true;
                bool isNotPathable = true;
                int speed = 100;
                int floorChange = Otc::FloorChangeNone;

                if(g_map.isAwareOfPosition(neighborPos)) {
                    wasSeen = true;
                    if(const TilePtr& tile = getTile(neighborPos)) {
                        hasCreature = tile->hasCreature();
                        isNotWalkable = !tile->isWalkable(flags & Otc::PathFindAllowCreatures);
                        isNotPathable = !tile->isPathable();
                        speed = tile->getGroundSpeed();
                    }
                } else {
                    const MinimapTile& mtile = g_minimap.getTile(neighborPos);
                    wasSeen = mtile.hasFlag(MinimapTileWasSeen);
                    isNotWalkable = mtile.hasFlag(MinimapTileNotWalkable);
                    isNotPathable = mtile.hasFlag(MinimapTileNotPathable);
                    if(isNotWalkable || isNotPathable)
                        wasSeen = true;
                    speed = mtile.getSpeed();
                    floorChange = mtile.getFloorChange();
                }

                if(!(flags & Otc::PathFindAllowNotSeenTiles) && !wasSeen)
                    continue;

                if(wasSeen && !(flags & Otc::PathFindAllowNonWalkable) && isNotWalkable)
                    continue;
                
                if (floorChange != Otc::FloorChangeNone && neighborPos != goalPos && ((flags & Otc::PathFindMultiFloor) || !(floorChange & Otc::FloorChangeAction))) {
                    // we probably don't want to change floor if PathFindMultiFloor is not given
                    if (!(flags & Otc::PathFindMultiFloor))
                        continue;
                }

                if(neighborPos != goalPos) {
                    if(wasSeen) {
                        if(!(flags & Otc::PathFindAllowCreatures) && hasCreature)
                            continue;

                        // allow walking over non-pathable stairs
                        if(!(flags & Otc::PathFindAllowNonPathable) && isNotPathable && (!(flags & Otc::PathFindMultiFloor) || floorChange == Otc::FloorChangeNone))
                            continue;
                    }
                }

                float walkFactor = 0;
                if(walkDir >= Otc::NorthEast)
                    walkFactor += 3.0f;
                else
                    walkFactor += 1.0f;

                float cost = currentNode->cost + (speed * walkFactor) / 100.0f;

                Node *neighborNode;
                if(nodes.find(neighborPos) == nodes.end()) {
                    neighborNode = new Node(neighborPos);
                    nodes[neighborPos] = neighborNode;
                } else {
                    neighborNode = nodes[neighborPos];
                    if(neighborNode->cost <= cost)
                        continue;
                }

                neighborNode->prev = currentNode;
                neighborNode->cost = cost;
                neighborNode->totalCost = neighborNode->cost + neighborPos.distance(goalPos);
                neighborNode->dir = walkDir;
                searchList.push(std::make_pair(neighborNode, neighborNode->totalCost));
            }
        }

        if(!searchList.empty()) {
            currentNode = searchList.top().first;
            searchList.pop();
        } else
            currentNode = nullptr;
    }

    if(foundNode) {
        currentNode = foundNode;
        while(currentNode) {
            Node *previousNode = currentNode->prev;
            if (previousNode && currentNode->pos.z != previousNode->pos.z) {
                dirs.clear();

            } 
            dirs.push_back(currentNode->dir);

            currentNode = currentNode->prev;
        }
        dirs.pop_back();
        std::reverse(dirs.begin(), dirs.end());
        result = Otc::PathFindResultOk;
    }

    for(auto it : nodes)
        delete it.second;

    return ret;
}



bool Map::checkSightLine(const Position& fromPos, const Position& toPos)
{
    uint16 startx = fromPos.x, starty = fromPos.y, startz = fromPos.z;
    uint16 endx = toPos.x, endy = toPos.y, endz = toPos.z;

    int32_t x, y, z, dx = std::abs(startx - endx), dy = std::abs(starty - endy),
        dz = std::abs(startz - endz), sx, sy, sz, ey, ez, max = dx, dir = 0;
    if(dy > max)
    {
        max = dy;
        dir = 1;
    }

    if(dz > max)
    {
        max = dz;
        dir = 2;
    }

    switch(dir)
    {
        case 1:
            //x -> y
            //y -> x
            //z -> z
            std::swap(startx, starty);
            std::swap(endx, endy);
            std::swap(dx, dy);
            break;
        case 2:
            //x -> z
            //y -> y
            //z -> x
            std::swap(startx, startz);
            std::swap(endx, endz);
            std::swap(dx, dz);
            break;
        default:
            //x -> x
            //y -> y
            //z -> z
            break;
    }

    sx = ((startx < endx) ? 1 : -1);
    sy = ((starty < endy) ? 1 : -1);
    sz = ((startz < endz) ? 1 : -1);

    ey = ez = 0;
    x = startx;
    y = starty;
    z = startz;

    int32_t lastrx = 0, lastry = 0, lastrz = 0;
    for(; x != endx + sx; x += sx)
    {
        int32_t rx, ry, rz;
        switch(dir)
        {
            case 1:
                rx = y; ry = x; rz = z;
                break;
            case 2:
                rx = z; ry = y; rz = x;
                break;
            default:
                rx = x; ry = y; rz = z;
                break;
        }

        if(!lastrx && !lastry && !lastrz)
        {
            lastrx = rx;
            lastry = ry;
            lastrz = rz;
        }

        if(lastrz != rz || ((toPos.x != rx || toPos.y != ry || toPos.z != rz) && (fromPos.x != rx || fromPos.y != ry || fromPos.z != rz)))
        {
            if(lastrz != rz && getTile({(uint16)lastrx, (uint16)lastry, (uint8)std::min(lastrz, rz)}))
                return false;

            lastrx = rx; lastry = ry; lastrz = rz;
            TilePtr tile = getTile({(uint16)rx, (uint16)ry, (uint8)rz});
            if (tile && !tile->isLookPossible())
                return false;
        }

        ey += dy;
        ez += dz;
        if(2 * ey >= dx)
        {
            y += sy;
            ey -= dx;
        }

        if(2 * ez >= dx)
        {
            z += sz;
            ez -= dx;
        }
    }

    return true;
}

bool Map::isSightClear(const Position& fromPos, const Position& toPos, bool floorCheck)
{
    if(floorCheck && fromPos.z != toPos.z)
        return false;

    // Cast two converging rays and see if either yields a result.
    return checkSightLine(fromPos, toPos) || checkSightLine(toPos, fromPos);
}


TilePtr Map::getBestDistanceTile(std::vector<CreaturePtr> creatures, int distance, bool autoWalk = false, bool walkNonPathable = false)
{
    // pathfinding using A* search algorithm
    // as described in http://en.wikipedia.org/wiki/A*_search_algorithm
    if (creatures.empty()) {
        return nullptr;
    }

    LocalPlayerPtr player = g_game.getLocalPlayer();
    Position playerPos = player->getPosition();
    struct Node {
        Node(const Position& pos) : cost(1e6), creatureCost(1e6), currentCreatureCost(1e6), potential(1e6), minDistance(1e6), pos(pos), prev(nullptr), dir(Otc::InvalidDirection) { }
        float cost;
        float creatureCost;
        float currentCreatureCost;
        float potential;
        float minDistance;
        inline float getDistance() const {
            return getDistance(cost, minDistance);
        };
        static float getDistance(const float &cost, const float &minDistance) {
            return cost - minDistance*0.9;
        };
        Position pos;
        Node *prev;
        Otc::Direction dir;
    };

    const int MAX_DISTNACE = 14;

    struct LessNode : std::binary_function<std::pair<Node*, float>, std::pair<Node*, float>, bool> {
        bool operator()(std::pair<Node*, float> a, std::pair<Node*, float> b) const {
            return b.second < a.second;
        }
    };
    std::vector<Node*> nodesByDistance[1000];

    std::unordered_map<Position, Node*, PositionHasher> nodes;
    std::priority_queue<std::pair<Node*, float>, std::vector<std::pair<Node*, float>>, LessNode> searchList;

    std::set<Position> unawarePositions;

    Node *currentNode = nullptr;
    for (auto creature : creatures) {
        if (!creature->getSpeed()) // do not count creatures standing still, or we'll get NaN
            continue;

        std::vector<Node*> nodesToClear;

        const Position creaturePos = creature->getPosition();
        if(nodes.find(creaturePos) == nodes.end()) {
            currentNode = new Node(creaturePos);
            nodes[creaturePos] = currentNode;
        } else {
            currentNode = nodes[creaturePos];
        }
        currentNode->currentCreatureCost = 0;

        while(currentNode) { // calculating creatureCost
            nodesToClear.push_back(currentNode);

            for(int i=-1;i<=1;++i) {
                for(int j=-1;j<=1;++j) {
                    if(i == 0 && j == 0)
                        continue;

                    Position neighborPos = currentNode->pos.translated(i, j);

                    int sqDistance = std::max(std::abs(playerPos.x - neighborPos.x), std::abs(playerPos.y - neighborPos.y));
                    if (sqDistance >= MAX_DISTNACE)
                        continue;

                    const TilePtr& tile = getTile(neighborPos);
                    if (!tile) {
                        unawarePositions.insert(neighborPos);
                    }
                    if(!tile || !tile->isWalkable(true))
                        continue;

                    float walkFactor = currentNode->pos.getDirectionFromPosition(neighborPos) >= Otc::NorthEast ? 3.0f : 1.0f;
                    float cost = currentNode->currentCreatureCost + (tile->getGroundSpeed() * walkFactor) / (float)creature->getSpeed() * (float)player->getSpeed() / 100.0f;

                    Node *neighborNode;
                    if(nodes.find(neighborPos) == nodes.end()) {
                        neighborNode = new Node(neighborPos);
                        nodes[neighborPos] = neighborNode;
                    } else {
                        neighborNode = nodes[neighborPos];
                        if(neighborNode->currentCreatureCost <= cost) {
                            continue;
                        }
                    }

                    neighborNode->currentCreatureCost = cost;
                    if (neighborNode->creatureCost > cost)
                        neighborNode->creatureCost = cost;

                    searchList.push(std::make_pair(neighborNode, neighborNode->currentCreatureCost));
                }
            }

            if(!searchList.empty()) {
                currentNode = searchList.top().first;
                searchList.pop();
            } else
                currentNode = nullptr;
        }

        for (auto node : nodesToClear) 
            node->currentCreatureCost = 1e6;

        nodesToClear.clear();
    }

    for (auto it : nodes) {
        if(int(it.second->creatureCost) < 1000)
            nodesByDistance[int(it.second->creatureCost)].emplace_back(it.second);
    }
    int distances = 2;

    for (int distance = 1000-1; distance >= 0; distance--) {
        if (!nodesByDistance[distance].empty()) {
            if (distances-- > 0) {
                for (auto node : nodesByDistance[distance]) {
                    if (currentNode)
                        searchList.push(std::make_pair(currentNode, 0));

                    currentNode = node;
                    currentNode->potential = 0;
                }


            }
        }
    }
    for (auto it : nodesByDistance)
        it.clear();

    for (const auto &pos : unawarePositions) {
        if (currentNode)
            searchList.push(std::make_pair(currentNode, 0));
        if(nodes.find(pos) == nodes.end()) {
            currentNode = new Node(pos);
            nodes[pos] = currentNode;
        } else {
            currentNode = nodes[pos];
        }
        currentNode->potential = 0;
    }

    while(currentNode) { // calculating runaway potential
        for(int i=-1;i<=1;++i) {
            for(int j=-1;j<=1;++j) {
                if(i == 0 && j == 0)
                    continue;

                Position neighborPos = currentNode->pos.translated(i, j);
                int sqDistance = std::max(std::abs(playerPos.x - neighborPos.x), std::abs(playerPos.y - neighborPos.y));
                if (sqDistance >= MAX_DISTNACE)
                    continue;

                const TilePtr& tile = getTile(neighborPos);
                if(!tile || !tile->isWalkable(neighborPos == playerPos)) // ignore creatures only if it is current player pos
                    continue;

                float walkFactor = currentNode->pos.getDirectionFromPosition(neighborPos) >= Otc::NorthEast ? 3.0f : 1.0f;

                float cost = currentNode->potential + (tile->getGroundSpeed() * walkFactor) / 100.0f;

                Node *neighborNode;
                if(nodes.find(neighborPos) == nodes.end()) {
                    neighborNode = new Node(neighborPos);
                    nodes[neighborPos] = neighborNode;
                } else {
                    neighborNode = nodes[neighborPos];
                    if(neighborNode->potential <= cost)
                        continue;
                }

                neighborNode->potential = cost;
                searchList.push(std::make_pair(neighborNode, neighborNode->potential));
            }
        }

        if(!searchList.empty()) {
            currentNode = searchList.top().first;
            searchList.pop();
        } else
            currentNode = nullptr;
    }

    if(nodes.find(playerPos) == nodes.end()) {
        currentNode = new Node(playerPos);
        nodes[playerPos] = currentNode;
    } else {
        currentNode = nodes[playerPos];
    }

    Node* currentPosNode = currentNode;

    currentNode->cost = 0;
    currentNode->minDistance = currentNode->creatureCost;

    while(currentNode) {
        for(int i=-1;i<=1;++i) {
            for(int j=-1;j<=1;++j) {
                if(i == 0 && j == 0)
                    continue;

                Position neighborPos = currentNode->pos.translated(i, j);

                int sqDistance = std::max(std::abs(playerPos.x - neighborPos.x), std::abs(playerPos.y - neighborPos.y));
                if (sqDistance >= MAX_DISTNACE)
                    continue;

                const TilePtr& tile = getTile(neighborPos);
                if(!tile) 
                    continue;

                // ignore not walkable tiles, not pathable tiles and floorchange tiles that do not require action
                if(!tile->isWalkable() || (!walkNonPathable && !tile->isPathable()) || (tile->getFloorChange() && !(tile->getFloorChange() & Otc::FloorChangeAction)))
                    continue;

                
                Otc::Direction walkDir = currentNode->pos.getDirectionFromPosition(neighborPos);

                float walkFactor = (walkDir >= Otc::NorthEast) ? 3.0f : 1.0f;
                float cost = currentNode->cost + (tile->getGroundSpeed() * walkFactor) / 100.0f;

                Node *neighborNode;
                if(nodes.find(neighborPos) == nodes.end()) {
                    neighborNode = new Node(neighborPos);
                    nodes[neighborPos] = neighborNode;
                } else {
                    neighborNode = nodes[neighborPos];   
                }
                float minDistance = std::min(currentNode->minDistance, neighborNode->creatureCost - cost);

                if(neighborNode->prev && neighborNode->getDistance() <= Node::getDistance(cost, minDistance))
                    continue;

                neighborNode->prev = currentNode;
                neighborNode->cost = cost;
                neighborNode->dir = walkDir;
                neighborNode->minDistance = minDistance;

                searchList.push(std::make_pair(neighborNode, neighborNode->getDistance()));
            }
        }

        if(!searchList.empty()) {
            currentNode = searchList.top().first;
            searchList.pop();
        } else
            currentNode = nullptr;
    }
    Node *bestNode = NULL;
    float bestNodeScore = 1e7;
    for (auto &it : nodes) {
        Node *node = it.second;
        if (node->cost >= 1e6) continue;
        float score = node->cost + node->potential*1.5 + (currentPosNode->minDistance - node->minDistance)*2;

        int sqDistance = 1<<30;
        for (auto creature : creatures) {
            const Position creaturePos = creature->getPosition();
            int creatureDist = std::max(std::abs(creaturePos.x - it.first.x), std::abs(creaturePos.y - it.first.y));
            if (creatureDist < sqDistance)
                sqDistance = creatureDist;
        }    
        score += std::abs(sqDistance - distance)*4;

        if (!isSightClear(node->pos, creatures[0]->getPosition()))
            score += 3;

        if (!bestNode || score < bestNodeScore) {
            bestNode = node;
            bestNodeScore = score;
        }

        /*std::ostringstream scoreText;
        score -= score - ((int)(score*10))/10.f;
        scoreText << std::defaultfloat << score;
        bool mustAdd = true;
        for(auto other : m_staticTexts) {
            // try to combine messages
            if(other->getPosition() == node->pos) {
                other->setText(scoreText.str());
                mustAdd = false;
                break;
            }
        }
        if (mustAdd) {
            StaticTextPtr staticText = StaticTextPtr(new StaticText);
            staticText->addMessage("", Otc::MessageBarkLoud, scoreText.str());
            g_map.addThing(staticText, node->pos, -1);
        }*/
    }

    TilePtr ret = getTile(bestNode->pos);
    if (autoWalk) {
        currentNode = bestNode;
        std::vector<Otc::Direction> dirs;

        while(currentNode) {
            dirs.push_back(currentNode->dir);
            currentNode = currentNode->prev;
        }
        dirs.pop_back();
        std::reverse(dirs.begin(), dirs.end());
        if (dirs.size()) {
            g_game.manualWalk(dirs);
        }
    }

    for(auto it : nodes)
        delete it.second;

    return ret;
}
