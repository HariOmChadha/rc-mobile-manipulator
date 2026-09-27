"""Review saved episodes after recording resources have closed."""

import argparse
import sys

from .recording import classify_episode


def review_result(result, root, quality="ask"):
    if quality == "ask":
        quality = "unreviewed"
        if sys.stdin.isatty():
            try:
                while True:
                    answer = (
                        input("Was this run good or bad? [g/b] (Enter keeps it unreviewed): ").strip().lower()
                    )
                    if answer in ("g", "good", "b", "bad", ""):
                        quality = {"g": "good", "good": "good", "b": "bad", "bad": "bad", "": "unreviewed"}[
                            answer
                        ]
                        break
                    print("Enter g for good, b for bad, or press Enter to review later.")
            except (EOFError, KeyboardInterrupt):
                print("\nKept unreviewed.")
        else:
            print(
                "No interactive terminal: kept unreviewed. Use ./robot classify EPISODE_PATH good|bad later.",
                file=sys.stderr,
            )
    path = classify_episode(result["path"], quality, root)
    return {**result, "path": str(path), "quality": quality}


def main():
    parser = argparse.ArgumentParser(description="Classify an existing saved episode")
    parser.add_argument("episode")
    parser.add_argument("quality", choices=["good", "bad", "unreviewed"])
    parser.add_argument(
        "--output", default="training_dataset", help="Dataset root (default: training_dataset)"
    )
    args = parser.parse_args()
    try:
        print(f"Saved {args.quality} episode: {classify_episode(args.episode, args.quality, args.output)}")
    except (OSError, ValueError) as error:
        parser.exit(1, f"Classification failed; inspect the original episode: {error}\n")


if __name__ == "__main__":
    main()
