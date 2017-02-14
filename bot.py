import hlt
from hlt import NORTH, EAST, SOUTH, WEST, STILL, Move, Square, opposite_cardinal
import heapq
import collections
import math
import random


myID, game_map = hlt.get_init()
n_squares = game_map.width * game_map.height
diameter = math.sqrt(n_squares)
n_players = game_map.starting_player_count

enemy_bases = [sq for sq in game_map if sq.owner not in (0, myID)]

# initialization
hlt.send_init("schmit")
frame = 0

def dijkstra_grid(queue, cost):
    """
    Wrap around Dijkstra shortest path BFS based on cost matrix "cost"
    where we find the shortest paths to list "start"

    Note in this case we are searching for a backwards path:
    the shortest path to the "start" node, rather than the other way around

    queue: list of tuples with (initial_cost, squares) to which to search
    cost: function(square) -> cost
    """
    heapq.heapify(queue)

    dist = collections.defaultdict(lambda: 1e9)
    bestdir = collections.defaultdict(lambda: 0)

    for initial_cost, square in queue:
        dist[(square.x, square.y)] = initial_cost

    while queue:
        du, square = heapq.heappop(queue)

        # for each neighbor
        for direction, target in enumerate(game_map.neighbors(square)):
            new_distance = du + cost(square)

            if new_distance < dist[(target.x, target.y)]:
                dist[(target.x, target.y)] = new_distance
                # flip the direction
                bestdir[(target.x, target.y)] = opposite_cardinal(direction)

                # update queue by removing the (old) newVertex and inserting the updated newVertex
                queue = [(du, square) for du, square in queue if square != target]
                heapq.heappush(queue, (new_distance, target))

    return dist, bestdir

def my_production():
    return sum(sq.production for sq in game_map if sq.owner == myID)

def my_perimeter():
    return (sq for sq in game_map if sq.owner == myID and any(n.owner != myID for n in game_map.neighbors(sq)))


def overkill_heuristic(square):
    # return total potential damage caused by overkill when attacking this square
    return sum(neighbor.strength + neighbor.production
            for neighbor in game_map.neighbors(square, n=2, include_self=True)
            if neighbor.owner not in (0, myID))

def prod_eff(square):
    return square.production**1.8 / (square.strength + 5)

def tunnel_targets(game_map):
    n_squares = game_map.width * game_map.height
    n_targets = int(max(10, 4*n_squares ** 0.7))
    value = lambda sq: prod_eff(sq)*(sq.owner==0) + sum(prod_eff(n) * (n.owner == 0)
                    for n in game_map.neighbors(sq, include_self=True))

    sorted_squares = sorted((square for square in game_map if square.owner == 0), key=value, reverse=True)

    return sorted_squares

def enemy_border(game_map):
    """ return squares with little to no strength that border both
    my territory, and an enemy's territory """
    no_mans_land = (sq for sq in game_map if sq.owner == 0 and sq.strength < 500)
    targets = [sq for sq in no_mans_land
            if (any(n.owner == myID
                    for n in game_map.neighbors(sq)) and
                any(n.owner > 0 and n.owner != myID
                    for n in game_map.neighbors(sq)))]
    return targets

def action_sequence(actions, square):
    if actions == []:
        return Move(square, STILL)

    action = actions.pop(0)
    move = action(square)
    if move is not None:
        return move
    return action_sequence(actions, square)

def max_neighbor(square, condition, key):
    return max(((neighbor, direction) for direction, neighbor in enumerate(game_map.neighbors(square))
                                if condition(neighbor)),
                                default = (None, None),
                                key = lambda t: key(t[0]))

def overkill(square):
    target, direction = max_neighbor(square,
            lambda sq: sq.owner != myID,
            lambda sq: overkill_heuristic(sq))
    if target is not None and overkill_heuristic(target) > 1 and target.strength < square.strength:
        return Move(square, direction)

def defend_battleground(square):
    if battlegrounds != []:
        direction = bg_dirs[(square.x, square.y)]
        distance = bg_dists[(square.x, square.y)]
        if distance < diameter / 5:
            return Move(square, direction)


def expand(square):
    """ expand if there is a lot of local capacity """
    target, direction = max_neighbor(square,
            lambda sq: sq.owner != myID,
            prod_eff)

    if target is not None and (square.strength > 1.5*target.strength or square.strength > 180):
        return Move(square, direction)


def strengthen(square):
    perimeter_capacity = my_peri_size * 255
    production_needed = perimeter_capacity * n_players / (n_players+1)
    # how many time steps needed for production to fill entire perimeter
    production_time = production_needed / my_prod

    if frame > 20:
        clipped_prod_factor = min(7, max(1, production_time))
    else:
        clipped_prod_factor = 4.5

    if square.strength < max(1, square.production * clipped_prod_factor):
        return Move(square, STILL)

def tunnel(square):
    direction = tunnel_dirs[(square.x, square.y)]
    target = game_map.get_target(square, direction)

    if square.strength > target.strength or target.owner == myID:
        return Move(square, tunnel_dirs[(square.x, square.y)])

def get_move(square):
    move = action_sequence([overkill,
                            strengthen,
                            defend_battleground,
                            expand,
                            tunnel
                            ], square)
    return move


def tunnel_cost(square):
    cost = 2.0

    # cost of unowned territory
    if square.owner != myID:
        cost += 4 * square.strength / (square.production+1)

    # cost of loss of production
    if square.owner == myID:
        cost += 0.3 * square.production
        cost += 0.02 * square.strength

    # cost of moving internally
    if all(n.owner == myID for n in game_map.neighbors(square)):
        cost += 4

    return cost

def post_process(moves):
    strengths = {(sq.x, sq.y): sq.strength * (2*(sq.owner == myID) - 1)
            for sq in game_map}
    ordered_by_dist_to_border = sorted(moves,
            key=lambda move: neutral_dists[(move.square.x, move.square.y)])

    new_moves = []
    for move in ordered_by_dist_to_border:
        target = game_map.get_target(move.square, move.direction)
        target_str = strengths[(target.x, target.y)]
        if target_str + move.square.strength > 285:
            new_moves.append(Move(move.square, STILL))
        else:
            new_moves.append(move)
            strengths[(target.x, target.y)] += move.square.strength
            strengths[(move.square.x, move.square.y)] -= move.square.strength

    return new_moves

while True:
    frame += 1
    game_map.get_frame()

    # find distance to enemies
    # enemy_dists, enemy_dirs = dijkstra_grid(
    #         [(0, sq) for sq in game_map if sq.owner not in (0, myID)],
    #         lambda sq: 1)

    neutral_dists, neutral_dirs = dijkstra_grid(
            [(0, sq) for sq in game_map if sq.owner == 0],
            lambda sq: 1)

    # battlegrounds
    battlegrounds = enemy_border(game_map)
    if battlegrounds != []:
        bg_dists, bg_dirs = dijkstra_grid(
                [(0, sink) for sink in battlegrounds],
                lambda sq: 1 + 5 * sq.strength/255 * (0.1 + 0.9 * (sq.owner != myID)))

    # tunnels to hight production areas
    tunnel_sinks = tunnel_targets(game_map)
    tunnel_dists, tunnel_dirs = dijkstra_grid(
            [(3*i/n_squares, sink) for i, sink in enumerate(tunnel_sinks)],
            tunnel_cost)

    my_prod = my_production()
    my_peri_size = sum(1 for s in my_perimeter())
    prelim_moves = [get_move(square) for square in game_map if square.owner == myID]

    final_moves = post_process(prelim_moves)
    hlt.send_frame(final_moves)

