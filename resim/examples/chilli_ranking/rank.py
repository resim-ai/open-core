#!/usr/bin/env python3
"""Interactive CLI Chilli Ranker

This script allows multiple people to rank chilli recipes across 5 categories:
- Visual (1-5) - How good does it look?
- Aroma (1-5) - How good does it smell?
- Texture (1-5) - How's the consistency and mouthfeel?
- Taste (1-5) - How does it taste?
- Spice Level (1-5) - How spicy is it?

Each chilli recipe is created as a separate test, and all ratings are emitted as metrics.

The script can export ratings to CSV and reload them later to avoid re-entering data.
"""

import csv
import os
import sys
from pathlib import Path
from resim.sdk.sdk import init
from resim.metrics.python.emissions import emit


def get_rating(category: str, min_val: int = 1, max_val: int = 5) -> int:
    """Get a valid rating from the user."""
    while True:
        try:
            rating = int(input(f"  {category} ({min_val}-{max_val}): "))
            if min_val <= rating <= max_val:
                return rating
            else:
                print(f"  Please enter a number between {min_val} and {max_val}")
        except ValueError:
            print("  Please enter a valid number")
        except KeyboardInterrupt:
            print("\n\nRanking cancelled by user.")
            exit(0)


def get_yes_no(prompt: str) -> bool:
    """Get a yes/no response from the user."""
    while True:
        try:
            response = input(f"{prompt} (y/n): ").lower().strip()
            if response in ["y", "yes"]:
                return True
            elif response in ["n", "no"]:
                return False
            else:
                print("  Please enter 'y' or 'n'")
        except KeyboardInterrupt:
            print("\n\nRanking cancelled by user.")
            exit(0)


def save_to_csv(chilli_name: str, ratings: list[dict]) -> str:
    """Save ratings to a CSV file."""
    # Create a safe filename
    safe_name = "".join(
        c if c.isalnum() or c in (" ", "-", "_") else "_" for c in chilli_name
    )
    safe_name = safe_name.replace(" ", "_").lower()
    filename = f"chilli_ratings_{safe_name}.csv"

    with open(filename, "w", newline="", encoding="utf-8") as csvfile:
        fieldnames = [
            "chilli_name",
            "person_name",
            "visual",
            "aroma",
            "texture",
            "taste",
            "spice_level",
            "overall_score",
        ]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for rating in ratings:
            writer.writerow(
                {
                    "chilli_name": chilli_name,
                    "person_name": rating["person_name"],
                    "visual": rating["visual"],
                    "aroma": rating["aroma"],
                    "texture": rating["texture"],
                    "taste": rating["taste"],
                    "spice_level": rating["spice_level"],
                    "overall_score": rating["overall_score"],
                }
            )

    return filename


def load_from_csv(filename: str) -> tuple[str, list[dict]]:
    """Load ratings from a CSV file. Returns (chilli_name, ratings_list)."""
    ratings = []
    chilli_name = ""

    with open(filename, "r", newline="", encoding="utf-8") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            if not chilli_name:
                chilli_name = row["chilli_name"]

            ratings.append(
                {
                    "person_name": row["person_name"],
                    "visual": int(row["visual"]),
                    "aroma": int(row["aroma"]),
                    "texture": int(row["texture"]),
                    "taste": int(row["taste"]),
                    "spice_level": int(row["spice_level"]),
                    "overall_score": float(row["overall_score"]),
                }
            )

    if not chilli_name:
        raise ValueError("CSV file is empty or missing chilli_name")

    return chilli_name, ratings


def list_csv_files() -> list[str]:
    """List all CSV files in the current directory that match the pattern."""
    return [
        f
        for f in os.listdir(".")
        if f.startswith("chilli_ratings_") and f.endswith(".csv")
    ]


def collect_chilli_ratings(chilli_name: str) -> tuple[str, list[dict]]:
    """Collect ratings for a single chilli. Returns (chilli_name, ratings_list)."""
    print(f"\nRanking: {chilli_name}")
    print("-" * 60)

    # Collect ratings from multiple people
    all_ratings = []

    while True:
        try:
            person_name = input("\n👤 Enter rater's name: ").strip()
            if not person_name:
                print("  Name cannot be empty!")
                continue
        except KeyboardInterrupt:
            print("\n\nRanking cancelled by user.")
            return chilli_name, all_ratings

        # Check if this person made this chilli (can't rate their own)
        if get_yes_no(f"\nDid {person_name} make this chilli?"):
            print(f"  ⏭️  Skipping - {person_name} cannot rate their own chilli")
            if not get_yes_no("\nAdd another person's rating?"):
                break
            continue

        print(f"\n{person_name}'s ratings:")

        visual = get_rating("Visual")
        aroma = get_rating("Aroma")
        texture = get_rating("Texture")
        taste = get_rating("Taste")
        spice_level = get_rating("Spice Level")

        overall_score = visual + aroma + texture + taste + spice_level

        all_ratings.append(
            {
                "person_name": person_name,
                "visual": visual,
                "aroma": aroma,
                "texture": texture,
                "taste": taste,
                "spice_level": spice_level,
                "overall_score": overall_score,
            }
        )

        print(f"\n  Overall score: {overall_score}/25")

        if not get_yes_no("\nAdd another person's rating?"):
            break

    return chilli_name, all_ratings


def main():
    print("=" * 60)
    print("🍲  INTERACTIVE CHILLI RANKING SYSTEM 🍲")
    print("=" * 60)
    print()

    # Check for dry run mode
    dry_run = "--dry-run" in sys.argv or "-d" in sys.argv
    if dry_run:
        print("🔧 DRY RUN MODE: Emissions will be written locally, not synced to ReSim")
        print()

    # Ask if user wants to enter multiple chillies
    batch_mode = get_yes_no("Enter multiple chillies in one session?")
    print()

    # Collect chilli data
    all_chillies = []  # List of (chilli_name, ratings_list) tuples

    # Check if user wants to load from CSV
    csv_files = list_csv_files()
    if csv_files:
        print("📁 Found existing CSV files:")
        for i, f in enumerate(csv_files, 1):
            print(f"  {i}. {f}")
        print()

        if get_yes_no("Load data from an existing CSV file?"):
            # In batch mode, allow loading multiple CSVs
            while True:
                try:
                    choice = int(input(f"Enter file number (1-{len(csv_files)}): "))
                    if 1 <= choice <= len(csv_files):
                        csv_file = csv_files[choice - 1]
                        print(f"\n📂 Loading data from {csv_file}...")
                        chilli_name, all_ratings = load_from_csv(csv_file)
                        print(
                            f"✅ Loaded {len(all_ratings)} ratings for '{chilli_name}'"
                        )

                        all_chillies.append((chilli_name, all_ratings))
                        print_summary(chilli_name, all_ratings)

                        # In batch mode, ask if they want to load more
                        if batch_mode and get_yes_no("\nLoad another CSV file?"):
                            continue
                        else:
                            break
                    else:
                        print("Invalid choice.")
                        if not batch_mode:
                            print("Starting fresh entry.")
                            break
                except (ValueError, FileNotFoundError, KeyError) as e:
                    print(f"Error loading CSV: {e}")
                    if not batch_mode:
                        print("Starting fresh entry.")
                        break
                except KeyboardInterrupt:
                    print("\n\nRanking cancelled by user.")
                    if all_chillies:
                        break  # Continue to emission with what we have
                    else:
                        return

    if batch_mode:
        # Batch mode: collect multiple chillies (fresh entry)
        # Ask if they want to add fresh entries (in addition to any loaded CSVs)
        if all_chillies:
            if not get_yes_no("\n➕ Add fresh chilli entries (not from CSV)?"):
                # Skip to emission with loaded CSVs
                pass
            else:
                # Collect fresh entries
                while True:
                    try:
                        chilli_name = input(
                            "\nEnter the name of the chilli recipe to rank: "
                        ).strip()
                        if not chilli_name:
                            print("Chilli name cannot be empty!")
                            continue
                    except KeyboardInterrupt:
                        print("\n\nRanking cancelled by user.")
                        break

                    # Collect ratings for this chilli
                    chilli_name, ratings = collect_chilli_ratings(chilli_name)

                    # Check if we have any ratings
                    if not ratings:
                        print("\n⚠️  No ratings collected for this chilli. Skipping.")
                    else:
                        all_chillies.append((chilli_name, ratings))
                        print_summary(chilli_name, ratings)

                    # Ask if user wants to add another chilli
                    if not get_yes_no("\n➕ Add another chilli to this batch?"):
                        break
        else:
            # No CSVs loaded, collect fresh entries
            while True:
                try:
                    chilli_name = input(
                        "\nEnter the name of the chilli recipe to rank: "
                    ).strip()
                    if not chilli_name:
                        print("Chilli name cannot be empty!")
                        continue
                except KeyboardInterrupt:
                    print("\n\nRanking cancelled by user.")
                    break

                # Collect ratings for this chilli
                chilli_name, ratings = collect_chilli_ratings(chilli_name)

                # Check if we have any ratings
                if not ratings:
                    print("\n⚠️  No ratings collected for this chilli. Skipping.")
                else:
                    all_chillies.append((chilli_name, ratings))
                    print_summary(chilli_name, ratings)

                # Ask if user wants to add another chilli
                if not get_yes_no("\n➕ Add another chilli to this batch?"):
                    break

        if not all_chillies:
            print("\n⚠️  No chillies with ratings collected!")
            return

    else:
        # Single chilli mode
        if not all_chillies:
            # No CSV loaded, collect fresh entry
            try:
                chilli_name = input(
                    "Enter the name of the chilli recipe to rank: "
                ).strip()
                if not chilli_name:
                    print("Chilli name cannot be empty!")
                    return
            except KeyboardInterrupt:
                print("\n\nRanking cancelled by user.")
                return

            # Collect ratings for this chilli
            chilli_name, ratings = collect_chilli_ratings(chilli_name)

            # Check if we have any ratings
            if not ratings:
                print(
                    "\n⚠️  No ratings collected! At least one person must rate the chilli."
                )
                return

            all_chillies.append((chilli_name, ratings))
            print_summary(chilli_name, ratings)
        # else: CSV was loaded, all_chillies already populated

    # Save to CSV
    if get_yes_no("\n💾 Save ratings to CSV file(s)?"):
        for chilli_name, ratings in all_chillies:
            try:
                filename = save_to_csv(chilli_name, ratings)
                print(f"✅ Saved {chilli_name} to {filename}")
            except Exception as e:
                print(f"❌ Error saving {chilli_name}: {e}")

    # Emit data
    if dry_run:
        for chilli_name, ratings in all_chillies:
            emit_dry_run(chilli_name, ratings)
    else:
        emit_batch_to_resim(all_chillies)


def emit_dry_run(chilli_name: str, all_ratings: list[dict]):
    """Emit ratings data to local file (dry run mode)."""
    print("\n📝 Writing emissions to local file (dry run)...")

    # Create output directory if it doesn't exist
    output_dir = Path("./chilli_emissions")
    output_dir.mkdir(exist_ok=True)

    # Create a safe filename
    safe_name = "".join(
        c if c.isalnum() or c in (" ", "-", "_") else "_" for c in chilli_name
    )
    safe_name = safe_name.replace(" ", "_").lower()
    output_file = output_dir / f"{safe_name}_emissions.jsonl"

    # Emit individual ratings to local file (without validation)
    for rating in all_ratings:
        emit(
            topic_name="individual_rating",
            data={
                "chilli_name": chilli_name,
                "person_name": rating["person_name"],
                "visual": rating["visual"],
                "aroma": rating["aroma"],
                "texture": rating["texture"],
                "taste": rating["taste"],
                "spice_level": rating["spice_level"],
                "overall_score": rating["overall_score"],
            },
            file_path=output_file,
        )

    print(f"✅ Emissions written to {output_file}")
    print(f"   ({len(all_ratings)} ratings)")
    print("\n💡 To sync to ReSim, run without --dry-run flag")


def print_summary(chilli_name: str, all_ratings: list[dict]):
    """Print summary statistics for a chilli."""
    num_raters = len(all_ratings)
    avg_visual = sum(r["visual"] for r in all_ratings) / num_raters
    avg_aroma = sum(r["aroma"] for r in all_ratings) / num_raters
    avg_texture = sum(r["texture"] for r in all_ratings) / num_raters
    avg_taste = sum(r["taste"] for r in all_ratings) / num_raters
    avg_spice = sum(r["spice_level"] for r in all_ratings) / num_raters
    avg_overall = sum(r["overall_score"] for r in all_ratings) / num_raters

    print("\n" + "=" * 60)
    print(f"📊 SUMMARY FOR {chilli_name.upper()}")
    print("=" * 60)
    print(f"Number of raters: {num_raters}")
    print(f"Average Visual:      {avg_visual:.2f}/5")
    print(f"Average Aroma:       {avg_aroma:.2f}/5")
    print(f"Average Texture:     {avg_texture:.2f}/5")
    print(f"Average Taste:       {avg_taste:.2f}/5")
    print(f"Average Spice Level: {avg_spice:.2f}/5")
    print(f"Average Overall:     {avg_overall:.2f}/25")
    print("=" * 60)


def emit_batch_to_resim(all_chillies: list[tuple[str, list[dict]]]):
    """Emit ratings data for multiple chillies to ReSim in a single batch."""
    print(f"\n📤 Emitting data for {len(all_chillies)} chilli(es) to ReSim...")

    with init(
        project="chilli-ranking",
        system="i-love-chilli",
        batch="October Offsite Ranking pre-demo 1",
        test_suite="Ranktastic pre-demo-1",
        branch="main",
        version="1.0.0",
    ) as batch:
        for chilli_name, all_ratings in all_chillies:
            # Create a test for this chilli
            with batch.run_test(chilli_name) as test:
                # Emit individual ratings (aggregations are calculated in SQL)
                for rating in all_ratings:
                    test.emit(
                        "individual_rating",
                        {
                            "chilli_name": chilli_name,
                            "person_name": rating["person_name"],
                            "visual": rating["visual"],
                            "aroma": rating["aroma"],
                            "texture": rating["texture"],
                            "taste": rating["taste"],
                            "spice_level": rating["spice_level"],
                            "overall_score": rating["overall_score"],
                        },
                    )
            print(f"  ✅ Test '{chilli_name}' created")

    print("\n✅ All data successfully emitted!")
    print(f"   {len(all_chillies)} test(s) created in batch 'October Offsite Ranking'")


if __name__ == "__main__":
    main()
