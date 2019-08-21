/*
    This Program should simulate a A* pathfinding algorithem.
    Made By Racon
    06.08.2019

    ! UNFINISHED
    todo There should be an class above Node, where map, offset, etc. is stored, and the neighbour methods are implemented

*/

use std::collections::HashMap;

#[derive(Debug, Clone)]
struct Node {
    g_cost: i32,    //* Distance to Start Point
    h_cost: i32,    //* Distance to End Point
    f_cost: i32,    //* 'Value' of the Node
    x: i32,   //x Position 
    y: i32,   //y Position
}
impl Node {
    fn new(new_x: i32, new_y: i32, new_g_cost: i32) -> Node {
        let ret_node = Node {
            g_cost: new_g_cost,
            h_cost: 0,
            f_cost: i32::max_value(),
            x: new_x as i32,
            y: new_y as i32,
        };
        
        ret_node
    }

    fn update_f_cost(&mut self, end_node: &Node) {
        let dis_x = self.x - end_node.x;
        let dis_y = self.y - end_node.y;
        let smallest = if dis_x < dis_y { dis_x.abs() } else { dis_y.abs() };

        let new_h_cost =  14 * smallest + 10 * (dis_x.abs() - smallest + dis_y.abs() - smallest);

        if self.f_cost > new_h_cost + self.g_cost {
            self.f_cost = new_h_cost + self.g_cost;
            self.h_cost = new_h_cost;
        }
    }

    fn get_new_f_cost(end_node: &Node, new_node: &Node) -> i32 {
        let dis_x = new_node.x - end_node.x;
        let dis_y = new_node.y - end_node.y;
        let smallest = if dis_x < dis_y { dis_x.abs() } else { dis_y.abs() };
        
        14 * smallest + 10 * (dis_x.abs() - smallest + dis_y.abs() - smallest)
    }

    fn get_lowest(list: &HashMap<(i32, i32), Node>) -> (i32, i32) {
        let (old_key, old_value) = list.iter().nth(0).expect("get_lowest received an empty List");

        let mut lowest_node: &Node  = old_value;
        let mut lowest_key: &(i32, i32) = old_key;

        for (key, value) in list.iter() {
            if value.f_cost < lowest_node.f_cost {
                lowest_node = value;
                lowest_key = key;
            }
        }

        lowest_key.clone()
    }

    fn loop_neighbours(start_node: &Node, target_node: &Node, map: &[&[bool]], open_list: &mut HashMap<(i32, i32), Node>) -> HashMap<(i32, i32), Node> {
        const OFFSET: [(i32, i32); 8] = [(-1,-1), (0,-1), (1,-1), (-1,0), (1, 0), (-1,1), (0,1), (1,1)];
        let y_size = map.len() - 1;
        let x_size = map[0].len() - 1;

        if x_size == 0  || y_size == 0 {
            panic!("open_list is empty, or to small (in loop_neighbours)");
        }

        let mut closed_list = HashMap::new();
        closed_list.insert((start_node.x, start_node.y), start_node.clone());
        
        loop { 
            let current_node = open_list.remove( &Node::get_lowest(&open_list) ).expect("could not remove lowest value");
            closed_list.insert((current_node.x, current_node.y), current_node.clone());

            //check for end condition:
            if current_node.x == target_node.x && current_node.y == target_node.y {
                print!("Shoutest path has been found!");
                break;
            }

            if open_list.is_empty() {
                println!("No Path !");
                break;
            }

            
            //loop threw neighbours
            for &(x_off, y_off) in OFFSET.iter() {
                let mut neighbour: Node;
                if x_off == 0 || y_off == 0 {
                    neighbour = Node::new(current_node.x + x_off, current_node.y + y_off, current_node.g_cost + 10);
                }
                else {
                    neighbour = Node::new(current_node.x + x_off, current_node.y + y_off, current_node.g_cost + 14);
                }
                
                /* if neighbour is not a valid tile */
                if neighbour.x < 0 || neighbour.x >= x_size as i32 {
                    continue;
                }
                if neighbour.y < 0 || neighbour.y >= y_size as i32 {
                    continue;
                }
                if map[ (neighbour.x) as usize ][ (neighbour.y) as usize ] == true {
                    continue;
                }

                neighbour.update_f_cost(&target_node);
                
                /* if new path to neighbour is shorter || neighbour is not in open_list */
                if neighbour.f_cost > Node::get_new_f_cost(&target_node, &neighbour) || match open_list.get(&(neighbour.x, neighbour.y)) {Some(_x) => false, None => true,} {
                    //!^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Problem over here (is always true ??)
                    neighbour.update_f_cost(&target_node);
                    
                    /* neighbour is not in open_list */
                    if match open_list.get(&(neighbour.x, neighbour.y)) {Some(_x) => false, None => true,} {
                        open_list.insert((neighbour.x, neighbour.y), neighbour);
                    }
                }
            }
        }

        closed_list
    }

    fn init_neighbours(start_node: &Node, target_node: &Node, map: &[&[bool]]) ->HashMap<(i32, i32), Node> {
        const OFFSET: [(i32, i32); 8] = [(-1,-1), (0,-1), (1,-1), (-1,0), (1, 0), (-1,1), (0,1), (1,1)];
        let y_size = map.len() - 1;
        let x_size = map[0].len() - 1;

        if x_size == 0  || y_size == 0 {
            panic!("open_list is empty, or to small (in init_neighbours)");
        }

        let mut open_list = HashMap::new();

        //insert starting values into open list:
        for &(x_off, y_off) in OFFSET.iter() {
            let mut neighbour: Node;
            if x_off == 0 || y_off == 0 {
                neighbour = Node::new(start_node.x + x_off, start_node.y + y_off, 10);
            }
            else {
                neighbour = Node::new(start_node.x + x_off, start_node.y + y_off, 14);
            }

            if neighbour.x < 0 || neighbour.x >= x_size as i32 {
                continue;
            }
            if neighbour.y < 0 || neighbour.y >= y_size as i32 {
                continue;
            }
            if map[ (start_node.x + x_off) as usize ][ (start_node.y + y_off) as usize ] == true {
                continue;
            }

            neighbour.update_f_cost(&target_node);

            //add new node to open list
            open_list.insert((start_node.x + x_off, start_node.y + y_off), neighbour);
        }

        open_list
    }
}

fn main() {
    //todo better input interface
    //--------------------  
    const MAP: &[&[bool]] = &[
        &[false,false,false,false,false],
        &[false,false,false,false,false],
        &[false,false,false,false,false],
        &[false,false,false,false,false],
        &[false,false,false,false,false],
    ];
    let start_node:  Node = Node::new(0, 0, 0);
    let target_node: Node = Node::new(4, 4, 0);
    //--------------------

    //init open list with neighbours from the starting node
    let mut open_list = Node::init_neighbours(&start_node, &target_node, MAP);
    
    //cycle threw every neighbour, and go to the neighbour with lowest f_cost, until  target_node is reached //! return Option with Node, if no Path could be found
    let closed_list = Node::loop_neighbours(&start_node, &target_node, MAP, &mut open_list);    

    //Print out the shortest path
    println!("{:?}", closed_list);

    for (y, map_slice) in MAP.iter().enumerate() {
        for (x, &tile) in map_slice.iter().enumerate() {
            if  match closed_list.get(&(x as i32, y as i32)) {Some(_x) => true, None => false,} {
                print!("| X ");
            }
            else {
                print!("| {} ", tile as i8);
            }   
        }
        println!("");
    }
}
