
"""
Họ và tên: Phạm Quốc Việt
MSSV: 22119255
BT: Máy hút bụi - BFS, DFS
Link Github:
"""

import tkinter as tk
from tkinter import ttk
from collections import deque
import time
from typing import Tuple, List, FrozenSet, Set

# state
# obj1 = obj2 --> hash(obj1) = hash(obj2) --> obj1 và obj2 có thể thay thế cho nhau trong set/dict
# khi override __eq__ --> PHẢI override __hash__ để đảm bảo tính nhất quán khi dùng trong set/dict
class State:
    def __init__(self, robot_pos: Tuple[int, int], dirty_cells: FrozenSet):
        self.robot_pos = robot_pos
        self.dirty_cells = dirty_cells

    def __eq__(self, other):
        return self.robot_pos == other.robot_pos and self.dirty_cells == other.dirty_cells

    def __hash__(self):
        return hash((self.robot_pos, self.dirty_cells))


# node (state, parent, action, path_cost)
class Node:
    def __init__(self, state: State, parent=None, action: str = None, path_cost: int = 0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def get_full_path(self) -> List[dict]:
        path = []
        current = self

        # từ goad node truy vết ngược về root node
        while current is not None:
            path.append({
                "pos": current.state.robot_pos,
                "action": current.action,
                "cost": current.path_cost,
                "dirty_left": len(current.state.dirty_cells)
            })
            current = current.parent
        return path[::-1]


class VacuumWorld:
    def __init__(self):
        self.rows = 5
        self.cols = 5
        self.start_pos = (1, 1)
        self.obstacles = frozenset([(2, 2), (2, 3), (3, 1)])
        self.dirty = frozenset([(0, 3), (1, 4), (3, 3), (4, 1), (4, 4)])

    def is_valid(self, pos: Tuple[int,int]) -> bool:
        x, y = pos
        return 0 <= x < self.rows and 0 <= y < self.cols and pos not in self.obstacles

    def get_successors(self, state: State):
        """
        Tại vị trí hiện tại, tìm đường đi hợp lệ, lưu lại (state, action) tương ứng cho đường đi đó
        """
        x, y = state.robot_pos
        directions = [(-1, 0, "Up"), (1, 0, "Down"), (0, -1, "Left"), (0, 1, "Right")]
        successors = []

        for dx, dy, action in directions:
            # vị trí sau khi di chuyển theo direction (Up, Down, Left, Right)
            new_pos = (x + dx, y + dy)
            if self.is_valid(new_pos):
                # các ô bẩn
                new_dirty = set(state.dirty_cells)
                # nếu vị trí mới có bẩn thì dọn sạch 
                if new_pos in new_dirty:
                    new_dirty.remove(new_pos)
                # cập nhật lại state sau khi robot di chuyển và dọn sạch
                new_state = State(new_pos, frozenset(new_dirty))
                successors.append((new_state, action))
        return successors


# gui
class VacuumGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Model-Based Reflex Agent - 5x5 Grid")
        self.root.geometry("1150x700")
        
        self.world = VacuumWorld()
        self.setup_ui()
        self.draw_grid()

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        left = ttk.LabelFrame(main_frame, text="Button", padding=10)
        left.grid(row=0, column=0, sticky="ns", padx=(0,10))
        
        ttk.Button(left, text="BFS_1", width=22, 
                  command=lambda: self.start_search("BFS_1")).pack(pady=8, fill=tk.X)
        ttk.Button(left, text="BFS_2", width=22, 
                  command=lambda: self.start_search("BFS_2")).pack(pady=8, fill=tk.X)
        ttk.Button(left, text="DFS_1", width=22, 
                  command=lambda: self.start_search("DFS_1")).pack(pady=8, fill=tk.X)
        ttk.Button(left, text="DFS_2", width=22, 
                  command=lambda: self.start_search("DFS_2")).pack(pady=8, fill=tk.X)

        viz = ttk.LabelFrame(main_frame, text="Visualization", padding=10)
        viz.grid(row=0, column=1, sticky="nsew")
        self.canvas = tk.Canvas(viz, width=580, height=480, bg="#f8f9fa")
        self.canvas.pack()

        right = ttk.LabelFrame(main_frame, text="QT chạy (Frontier)", padding=10)
        right.grid(row=0, column=2, sticky="ns", padx=(10,0))
        self.queue_text = tk.Text(right, width=30, height=30, font=("Consolas", 9))
        self.queue_text.pack(fill=tk.BOTH, expand=True)

        bottom = ttk.LabelFrame(main_frame, text="Truy vết chuỗi (Path)", padding=10)
        bottom.grid(row=1, column=0, columnspan=3, sticky="ew", pady=10)
        self.path_text = tk.Text(bottom, height=9, font=("Consolas", 10))
        self.path_text.pack(fill=tk.BOTH, expand=True)

        main_frame.columnconfigure(1, weight=1)

    def draw_grid(self, dirty_cells=None, robot_pos=None):
        self.canvas.delete("all")
        cell_size = 85
        ox, oy = 40, 30
        current_dirty = dirty_cells if dirty_cells is not None else self.world.dirty
        pos = robot_pos or self.world.start_pos

        for i in range(self.world.rows):
            for j in range(self.world.cols):
                x1 = ox + j * cell_size
                y1 = oy + i * cell_size
                x2 = x1 + cell_size
                y2 = y1 + cell_size

                if (i, j) in self.world.obstacles:
                    color = "#555555"
                elif (i, j) in current_dirty:
                    color = "#ffaa00"
                else:
                    color = "#4ade80"

                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="#333333", width=3)
                self.canvas.create_text(x1 + cell_size//2, y1 + cell_size//2, 
                                      text=f"{i},{j}", font=("Arial", 10, "bold"))

        # Robot
        cx = ox + pos[1] * cell_size + cell_size//2
        cy = oy + pos[0] * cell_size + cell_size//2
        self.canvas.create_oval(cx-28, cy-28, cx+28, cy+28, fill="#2563eb", outline="white", width=6)

    # def update_frontier(self, frontier):
    #     self.queue_text.delete(1.0, tk.END)
    #     self.queue_text.insert(tk.END, f"Frontier size: {len(frontier)}\n\n")
    #     for i, node in enumerate(list(frontier)[:18]):
    #         self.queue_text.insert(tk.END, 
    #             f"{i+1:2d}. {node.state.robot_pos} | Dirt: {len(node.state.dirty_cells)}\n")


    def update_log(self, current: Node, is_new_step=True):
        """Hiển thị log tuần tự quá trình robot di chuyển"""
        if is_new_step:
            pos = current.state.robot_pos
            action = current.action or "START"
            
            # Dòng di chuyển
            line = f"Robot tới {pos} | {action}\n"
            self.queue_text.insert(tk.END, line)
            
            # Kiểm tra có hút bụi không
            if current.parent:
                dirty_before = len(current.parent.state.dirty_cells)
                dirty_now = len(current.state.dirty_cells)
                if dirty_now < dirty_before:
                    self.queue_text.insert(tk.END, f"  → HÚT BỤI tại {pos}\n")
            
            self.queue_text.insert(tk.END, "-" * 45 + "\n")
        
        # Cuộn xuống cuối để luôn thấy dòng mới nhất
        self.queue_text.see(tk.END)
        self.root.update()

    def print_path(self, goal_node: Node):
        path = goal_node.get_full_path()
        self.path_text.delete(1.0, tk.END)
        self.path_text.insert(tk.END, "ĐƯỜNG ĐI TỪ ROOT ĐẾN GOAL\n\n")
        print("\nĐƯỜNG ĐI ĐÃ TÌM ĐƯỢC")
        
        for i, step in enumerate(path):
            action = step["action"] or "START"
            action_str = f"{action:>6}" if action != "START" else " START"
            line = f"{i:2d}. Vị trí: {step['pos']} | Hành động: {action_str} | Cost: {step['cost']:2d} | Còn bẩn: {step['dirty_left']}\n"
            self.path_text.insert(tk.END, line)
            print(line.strip())

    # bfs1
    def bfs1(self, start_node: Node):
        frontier = deque([start_node])
        reached: Set[State] = set()

        if len(start_node.state.dirty_cells) == 0:
            self.print_path(start_node)
            self.draw_grid(start_node.state.dirty_cells, start_node.state.robot_pos)
            return

        while frontier:
            current = frontier.popleft()           

            reached.add(current.state)

            self.draw_grid(current.state.dirty_cells, current.state.robot_pos)
            self.update_log(current)
            self.root.update()
            time.sleep(0.09)

            # Goal Test
            if len(current.state.dirty_cells) == 0:
                self.print_path(current)
                self.draw_grid(current.state.dirty_cells, current.state.robot_pos)
                print("BFS_1 HOÀN THÀNH - Ma trận sạch!")
                return

            # Mở rộng các action
            for next_state, action in self.world.get_successors(current.state):
                child = Node(next_state, current, action, current.path_cost + 1)

                if child.state not in reached:
                    if not any(n.state == child.state for n in frontier):
                        frontier.append(child)

        print("BFS_1: Không tìm thấy giải pháp")


    def bfs2(self, start_node: Node):
        frontier = deque([start_node])
        reached: Set[State] = set()

        # Goal test initial node
        if len(start_node.state.dirty_cells) == 0:
            self.print_path(start_node)
            self.draw_grid(start_node.state.dirty_cells, start_node.state.robot_pos)
            return

        while frontier:
            current = frontier.popleft()

            reached.add(current.state)        

            self.draw_grid(current.state.dirty_cells, current.state.robot_pos)
            self.update_log(current)
            self.root.update()
            time.sleep(0.085)

            # Goal test trên node vừa dequeue
            if len(current.state.dirty_cells) == 0:
                self.print_path(current)
                self.draw_grid(current.state.dirty_cells, current.state.robot_pos)
                print("BFS_2 HOÀN THÀNH!")
                return

            # Generate children
            for next_state, action in self.world.get_successors(current.state):
                child = Node(next_state, current, action, current.path_cost + 1)

                # if child.STATE ∉ reached AND child ∉ frontier
                if child.state not in reached and not any(n.state == child.state for n in frontier):
                    
                    # Goal test trên child
                    if len(child.state.dirty_cells) == 0:
                        self.print_path(child)
                        self.draw_grid(child.state.dirty_cells, child.state.robot_pos)
                        print("BFS_2 HOÀN THÀNH (tìm thấy goal ở child)!")
                        return
                    
                    frontier.append(child)

        print("BFS_2: Không tìm thấy giải pháp")



    def dfs1(self, start_node: Node):
        frontier = [start_node]                    
        reached: Set[State] = set()              

        if len(start_node.state.dirty_cells) == 0:
            self.print_path(start_node)
            self.draw_grid(start_node.state.dirty_cells, start_node.state.robot_pos)
            print("DFS_1 HOÀN THÀNH (initial goal)!")
            return

        while frontier:
            current = frontier.pop()              

            reached.add(current.state)             

            self.draw_grid(current.state.dirty_cells, current.state.robot_pos)
            self.update_log(current)
            self.root.update()
            time.sleep(0.12)

            # Goal test
            if len(current.state.dirty_cells) == 0:
                self.print_path(current)
                self.draw_grid(current.state.dirty_cells, current.state.robot_pos)
                print("DFS_1 HOÀN THÀNH - Ma trận sạch!")
                return

            for next_state, action in self.world.get_successors(current.state):
                child = Node(next_state, current, action, current.path_cost + 1)

                if (child.state not in reached and 
                    not any(n.state == child.state for n in frontier)):
                    
                    frontier.append(child)        

        print("DFS_1: Không tìm thấy giải pháp")

    def dfs2(self, start_node: Node):
        frontier = [start_node]                    
        reached: Set[State] = set()              

        if len(start_node.state.dirty_cells) == 0:
            self.print_path(start_node)
            self.draw_grid(start_node.state.dirty_cells, start_node.state.robot_pos)
            print("DFS_2 HOÀN THÀNH (initial goal)!")
            return

        while frontier:
            current = frontier.pop()              

            reached.add(current.state)             

            self.draw_grid(current.state.dirty_cells, current.state.robot_pos)
            self.update_log(current)
            self.root.update()
            time.sleep(0.12)

            # Goal test
            if len(current.state.dirty_cells) == 0:
                self.print_path(current)
                self.draw_grid(current.state.dirty_cells, current.state.robot_pos)
                print("DFS_2 HOÀN THÀNH - Ma trận sạch!")
                return

            for next_state, action in self.world.get_successors(current.state):
                child = Node(next_state, current, action, current.path_cost + 1)

                if (child.state not in reached and 
                    not any(n.state == child.state for n in frontier)):
                    
                    if len(child.state.dirty_cells) == 0:
                        self.print_path(child)
                        self.draw_grid(child.state.dirty_cells, child.state.robot_pos)
                        print("DFS_2 HOÀN THÀNH (tìm thấy goal ở child)!")
                        return
                    
                    frontier.append(child)        

        print("DFS_2: Không tìm thấy giải pháp")




    def start_search(self, algo: str):
        self.queue_text.delete(1.0, tk.END)
        self.path_text.delete(1.0, tk.END)
        
        initial_state = State(self.world.start_pos, self.world.dirty)
        start_node = Node(initial_state)

        if algo == "BFS_1":
            self.bfs1(start_node)
        elif algo == "BFS_2":
            self.bfs2(start_node)   
        elif algo == "DFS_1":
            self.dfs1(start_node)
        elif algo == "DFS_2":
            self.dfs2(start_node)


    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = VacuumGUI()
    app.run()