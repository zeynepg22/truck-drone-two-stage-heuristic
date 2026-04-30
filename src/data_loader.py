import pandas as pd

def load_nodes(file_path):
    """
    Loads node data from CSV.

    Expected columns:
    node_id, x, y, type
    """
    df = pd.read_csv(file_path)

    required_columns = {"node_id", "x", "y", "type"}
    missing_columns = required_columns - set(df.columns)

    if missing_columns:
        raise ValueError(f"Missing columns in node file: {missing_columns}")

    return df