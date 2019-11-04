#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>
#include <queue>          

using namespace std;

namespace
{
	struct point_t
	{
		int x, y;
		
		inline bool operator==(point_t p) {
			return p.x == x && p.y == y;
		}
	};

	struct grid_t
	{
		enum cell_t
		{
			empty,
			wall,
		};

		int width, height;

		vector<cell_t> cells;

		point_t start;
		point_t target;

		int index_of(point_t const & point) const { return point.x + point.y * width; }
	};

	vector<string> read_lines(istream & is)
	{
		vector<string> lines;

		do
		{
			lines.emplace_back();
		}
		while (getline(is, lines.back()));

		lines.pop_back();

		return lines;
	}

	bool read_grid(istream & is, grid_t & grid)
	{
		vector<string> lines = read_lines(is);

		grid.height = lines.size();
		grid.width = accumulate(lines.begin(), lines.end(), 0, [](size_t width, auto const & line){ return max(width, line.size()); });

		grid.cells.resize(grid.width * grid.height, grid_t::empty);
		for (int y = 0; y < lines.size(); y++)
		{
			for (int x = 0; x < lines[y].size(); x++)
			{
				int const grid_index = grid.index_of({x, y});
				switch (lines[y][x])
				{
				case '#':
					grid.cells[grid_index] = grid_t::wall;
					break;
				case ' ':
					grid.cells[grid_index] = grid_t::empty;
					break;
				case 's':
					grid.cells[grid_index] = grid_t::empty;
					grid.start = point_t{x, y};
					break;
				case 't':
					grid.cells[grid_index] = grid_t::empty;
					grid.target = point_t{x, y};
					break;
				default:
					cerr << "invalid character at (" << x << ", " << y << ")\n";
					return false;
				}
			}
		}
		return true;
	}

	vector<point_t> flood_fill(grid_t const & grid)
	{
		vector<point_t> open_set;
		vector<point_t> closed_set;

		point_t current_pos = grid.start;
		open_set.push_back(current_pos);

		while (!open_set.empty())
		{
			// Check if target has been reached
			if (current_pos.x == grid.target.x && current_pos.y == grid.target.y) break;

			point_t up		= { current_pos.x, current_pos.y + 1 };
			point_t right	= { current_pos.x + 1, current_pos.y };
			point_t down	= { current_pos.x, current_pos.y - 1 };
			point_t left	= { current_pos.x - 1, current_pos.y };

			vector<point_t> neighbors = { up, right, down, left };

			for (int i = 0; i < neighbors.size(); i++)
			{
				int index = grid.index_of(neighbors[i]);
				if (index < grid.cells.size() && grid.cells[index] == grid.empty)
				{
					vector<point_t>::iterator it = find(closed_set.begin(), closed_set.end(), neighbors[i]); 
					if (it == closed_set.end())
					{
						it = find(open_set.begin(), open_set.end(), neighbors[i]);
						if (it == open_set.end())
							open_set.push_back(neighbors[i]);
					}	
				}
			}
			
			// Remove current node v
			open_set.erase(open_set.begin());
			
			// Add current node to path
			closed_set.push_back(current_pos);

			// Set next node
			if(!open_set.empty())
			current_pos = open_set.front();
		}
		return closed_set;
	}

	point_t get_point_from_index(int i, int w)
	{
		point_t point = { i % w, i / w };
		return point;
	}

	int calculate_heuristic(point_t start, point_t target)
	{
		return abs(start.x - target.x) + abs(start.y - target.y);
	}

	vector<point_t> priority_search(grid_t const& grid)
	{
		vector<point_t> path;
		vector<point_t> previous(grid.cells.size());
		vector<bool> visited(grid.cells.size(), false);
		
		vector<point_t> directions { 
			{1,0}, 
			{0,1},
			{-1,0},
			{0,-1}
		};

		point_t current = grid.target;
		priority_queue<pair<int, int>> next;
		next.push(make_pair(calculate_heuristic(current, grid.start), grid.index_of(current)));

		while (!next.empty())
		{
			current = get_point_from_index(next.top().second, grid.width);
			next.pop();
			visited[grid.index_of(current)] = true;

			if (grid.index_of(current) == grid.index_of(grid.start))
			{
				while (grid.index_of(current) != grid.index_of(grid.target))
				{
					point_t came_from = previous[grid.index_of(current)];
					path.push_back(came_from);
					current = came_from;
				}

				return path;
			}
			
			for (int i = 0; i < 4; i++)
			{
				point_t neighbor = { current.x + directions[i].x, current.y + directions[i].y };
				if (grid.cells[grid.index_of(neighbor)] == grid.wall || visited[grid.index_of(neighbor)] == true)
					continue;

				previous[grid.index_of(neighbor)] = current;
				visited[grid.index_of(neighbor)] = true;
				next.push(make_pair(calculate_heuristic(neighbor, grid.start), grid.index_of(neighbor)));
			}

		}

		return path;
	}

	struct intersection_t
	{
		enum : uint8_t { up = 1, left = 2, right = 4, down = 8, visited = 16 };

		uint8_t value = 0;
	};

	vector<intersection_t> compute_intersections(grid_t const & grid, vector<point_t> const & path)
	{
		vector<intersection_t> intersections(grid.width * grid.height);
		int prev_grid_index = -(grid.width + 1); // guaranteed not to be adjacent
		for (auto const & point : path)
		{
			bool const inside_grid =
				0 <= point.x && point.x < grid.width &&
				0 <= point.y && point.y < grid.height;
			if (!inside_grid)
			{
				prev_grid_index = -(grid.width + 1);
				continue;
			}

			int const grid_index = grid.index_of(point);
			intersections[grid_index].value = intersection_t::visited;

			if (grid_index == prev_grid_index - grid.width)
			{
				intersections[grid_index].value |= intersection_t::down;
				intersections[prev_grid_index].value |= intersection_t::up;
			}
			else if (grid_index == prev_grid_index - 1 && point.x < grid.width - 1)
			{
				intersections[grid_index].value |= intersection_t::right;
				intersections[prev_grid_index].value |= intersection_t::left;
			}
			else if (grid_index == prev_grid_index + 1 && point.x > 0)
			{
				intersections[grid_index].value |= intersection_t::left;
				intersections[prev_grid_index].value |= intersection_t::right;
			}
			else if (grid_index == prev_grid_index + grid.width)
			{
				intersections[grid_index].value |= intersection_t::up;
				intersections[prev_grid_index].value |= intersection_t::down;
			}
			prev_grid_index = grid_index;
		}
		return intersections;
	}

	void print_path(ostream & os, grid_t const & grid, vector<point_t> const & path)
	{
		vector<intersection_t> const intersections = compute_intersections(grid, path);
		for (int y = 0; y < grid.height; y++)
		{
			for (int x = 0; x < grid.width; x++)
			{
				int const grid_index = grid.index_of({x, y});
				switch (grid.cells[grid_index])
				{
				case grid_t::empty:
					if (grid.start.x == x && grid.start.y == y)
					{
						os << 's';
					}
					else if (grid.target.x == x && grid.target.y == y)
					{
						os << 't';
					}
					else
					{
						char const intersection_graphics[] = {
							' ', '?', '?', '?',
							'?', '?', '?', '?',
							'?', '?', '?', '?',
							'?', '?', '?', '?',

							'o', '|', '-', '/',
							'-', '\\', '-', '?',
							'|', '|', '\\', '?',
							'/', '?', '?', '?',
						};
						os << intersection_graphics[intersections[grid_index].value];
					}
					break;
				case grid_t::wall:
					os << (intersections[grid_index].value == 0 ? '#' : 'X');
					break;
				}
			}
			os << '\n';
		}
	}
}

int main(int argc, char const * const * const argv)
{
	grid_t grid;

	if (argc < 2)
	{
		if (!read_grid(cin, grid))
			return -1;
	}
	else
	{
		ifstream is(argv[1]);
		if (!is)
		{
			cerr << "input file not valid\n";
			return -1;
		}

		if (!read_grid(is, grid))
			return -1;
	}

	//vector<point_t> path = flood_fill(grid);
	vector<point_t> path = priority_search(grid);


	print_path(cout, grid, path);

	return 0;
}
