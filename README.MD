# Dijkstra Algorithm Implementation

This Python project implements Dijkstra's algorithm for finding shortest paths in graphs. It allows users to either create a new graph with specified nodes and edges or use prepared graphs from a dataset downloaded from [9th DIMACS Implementation Challenge](https://www.diag.uniroma1.it/challenge9/download.shtml) or any other source.

## Project Structure

```
.
├── LICENSE
├── README.md
├── decorators.py
├── graph_parser.py
├── main.py
├── requirements.txt
├── sample_filepaths.py
└── sample_graph_generator.py
```

### Files:

- **`decorators.py`**: Contains decorators for measuring execution time and handling errors.
- **`graph_parser.py`**: Parses graph data from files.
- **`main.py`**: Main script where users can choose to create a new graph or use a prepared graph to apply Dijkstra's algorithm.
- **`sample_filepaths.py`**: Template file for setting dataset paths.
- **`sample_graph_generator.py`**: Generates random graphs for testing purposes.

## Getting Started

### Prerequisites

Ensure Python 3.x is installed on your machine.

### Setting Up Your Dataset

1. **Rename the File**:
   - Rename `sample_filepaths.py` to `filepaths.py`.

2. **Set the Dataset Path**:
   - Open `filepaths.py` and set the `DATASET_PATH` variable to the location of your graph files.

## Usage

1. **Running the Program**:
   - Open a terminal or command prompt.
   - Navigate to the project directory.
   - Execute `main.py`:

     ```bash
     python main.py
     ```

2. **Choose Your Option**:
   - You will be prompted to choose:
     ```
     Enter your choice:
     1. Create a new graph
     2. Use a prepared graph
     Choice:
     ```

3. **Creating a New Graph**:
   - If you choose `1`, enter the number of nodes and edges for the new graph when prompted.

4. **Using a Prepared Graph**:
   - If you choose `2`, enter the path to the folder containing the graph files and the name of the graph files.

5. **Running Dijkstra's Algorithm**:
   - After choosing or generating a graph, enter the source vertex from which to compute shortest paths.

6. **Example Output**:
   - The program will display the computed distances from the source vertex to all other vertices, as well as the path through previous nodes.

### How It Works

- **`decorators.py`**: Provides decorators for measuring execution time and handling errors.
- **`graph_parser.py`**: Parses graph data from specified files.
- **`main.py`**: Offers an interactive interface for users to select graph creation or usage, applying Dijkstra's algorithm accordingly.
- **`sample_graph_generator.py`**: Generates random graphs for testing purposes.

## Graph Data Sources

This project utilizes graph data from the [9th DIMACS Implementation Challenge - Shortest Paths](https://www.diag.uniroma1.it/challenge9/download.shtml). You can use this dataset, generate new graphs using `sample_graph_generator.py`, or integrate any other graph dataset of your choice.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

---

Feel free to explore and utilize this tool for analyzing various graph datasets or testing different scenarios with Dijkstra's algorithm. Whether for educational purposes or practical applications, we hope you find this project valuable and adaptable to your needs. Happy pathfinding with Dijkstra's algorithm!