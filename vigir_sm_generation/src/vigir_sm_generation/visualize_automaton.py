import argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("json_path",
                        help="Path to the automaton .JSON file")
    parser.add_argument("save_dir",
                        help="Path to the directory where files are saved.")
    args = parser.parse_args()

    json_path = args.json_path
    save_dir = args.save_dir


if __name__ == '__main__':
    main()
