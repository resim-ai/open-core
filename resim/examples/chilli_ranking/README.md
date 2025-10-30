# 🍲 Interactive Chilli Ranking System

An interactive CLI tool for ranking chilli (the meal) across multiple categories with multiple raters, integrated with the ReSim metrics system.

## Features

- **5 Rating Categories**: Visual, Aroma, Texture, Taste, and Spice Level (1-5 scale)
- **Multiple Raters**: Collect ratings from as many people as you want
- **Self-Rating Prevention**: Automatically skips ratings when someone tries to rate their own chilli
- **Automatic Aggregation**: Calculates average scores across all raters
- **CSV Export/Import**: Save ratings to CSV and reload them later to avoid re-entering data
- **Dry Run Mode**: Write emissions locally without syncing to ReSim (great for testing)
- **ReSim Integration**: Emits data to ReSim for visualization and analysis
- **Rich Metrics**: Includes tables, bar charts, line charts, and scalar metrics

## Usage

### Normal Mode (Sync to ReSim)

Run the script to collect ratings and sync to ReSim:

```bash
python resim/examples/chilli_ranking/rank.py
```

### Dry Run Mode (Local Only)

Run with `--dry-run` or `-d` flag to write emissions locally without syncing to ReSim:

```bash
python resim/examples/chilli_ranking/rank.py --dry-run
# or
python resim/examples/chilli_ranking/rank.py -d
```

In dry run mode:
- Emissions are written to `./chilli_emissions/<chilli_name>_emissions.jsonl`
- No connection to ReSim is made
- Perfect for testing or offline data collection

The script will guide you through:

1. **Load from CSV (Optional)**: If you have existing CSV files, you can reload them
2. **Enter Chilli Name**: Provide the name of the chilli you're ranking
3. **Collect Ratings**: For each person:
   - Enter their name
   - Confirm if they made this chilli (they'll be skipped if yes - can't rate their own!)
   - Rate the chilli on:
     - Visual (1-5)
     - Aroma (1-5)
     - Texture (1-5)
     - Taste (1-5)
     - Spice Level (1-5)
4. **Add More Raters**: Choose whether to add another person's ratings
5. **View Summary**: See aggregate statistics for all raters (excluding self-ratings)
6. **Save to CSV (Optional)**: Export ratings to a CSV file for later use
7. **Automatic Emission**: Data is automatically sent to ReSim

## Example Session

```
============================================================
🍲  INTERACTIVE CHILLI RANKING SYSTEM 🍲
============================================================

Enter the name of the chilli to rank: Matt's Famous Chilli

Ranking: Matt's Famous Chilli
------------------------------------------------------------

👤 Enter rater's name: Sarah

Did Sarah make this chilli? (y/n): n

Sarah's ratings:
  Visual (1-5): 4
  Aroma (1-5): 3
  Texture (1-5): 4
  Taste (1-5): 5
  Spice Level (1-5): 3

  Overall score: 19/25

Add another person's rating? (y/n): y

👤 Enter rater's name: Matt

Did Matt make this chilli? (y/n): y
  ⏭️  Skipping - Matt cannot rate their own chilli

Add another person's rating? (y/n): y

👤 Enter rater's name: Alex

Did Alex make this chilli? (y/n): n

Alex's ratings:
  Visual (1-5): 3
  Aroma (1-5): 4
  Texture (1-5): 3
  Taste (1-5): 4
  Spice Level (1-5): 4

  Overall score: 18/25

Add another person's rating? (y/n): n

============================================================
📊 SUMMARY FOR MATT'S FAMOUS CHILLI
============================================================
Number of raters: 2
Average Visual:      3.50/5
Average Aroma:       3.50/5
Average Texture:     3.50/5
Average Taste:       4.50/5
Average Spice Level: 3.50/5
Average Overall:     18.50/25
============================================================

📤 Emitting data to ReSim...
✅ Data successfully emitted!

Test 'Matt's Famous Chilli' created in batch 'October Offsite Ranking'
```

## CSV Export/Import

### Saving Ratings to CSV

After entering all ratings, you'll be prompted to save to CSV:

```
💾 Save ratings to CSV file? (y/n): y
✅ Saved to chilli_ratings_matts_famous_chilli.csv
```

The CSV file contains all ratings with the following columns:
- `chilli_name`: Name of the chilli recipe
- `person_name`: Rater's name
- `visual`, `aroma`, `texture`, `taste`, `spice_level`: Individual category scores
- `overall_score`: Calculated average

### Loading Ratings from CSV

When you run the script, if CSV files exist in the current directory, you'll see:

```
📁 Found existing CSV files:
  1. chilli_ratings_matts_famous_chilli.csv
  2. chilli_ratings_sarahs_spicy_special.csv

Load data from an existing CSV file? (y/n): y
Enter file number (1-2): 1

📂 Loading data from chilli_ratings_matts_famous_chilli.csv...
✅ Loaded 12 ratings for 'Matt's Famous Chilli'
```

The script will then skip data entry and go straight to the summary and emission.

### Use Cases

- **Collect ratings offline**: Enter data on paper, then type it in once and save to CSV
- **Re-emit data**: If you need to re-run the batch, just reload the CSV files
- **Backup**: Keep CSV files as a backup of your ratings
- **Share data**: Send CSV files to others for their own analysis
- **Batch processing**: Collect all ratings first, then load and emit them all at once

## Dry Run Mode

Dry run mode allows you to test the system or collect data offline without connecting to ReSim.

### When to Use Dry Run

- **Testing**: Verify the script works before the actual event
- **Offline Collection**: No internet connection available
- **Preview**: See what emissions look like before syncing
- **Development**: Test changes without affecting ReSim data

### How It Works

```bash
python resim/examples/chilli_ranking/rank.py --dry-run
```

Output:
```
🍲  INTERACTIVE CHILLI RANKING SYSTEM 🍲
🔧 DRY RUN MODE: Emissions will be written locally, not synced to ReSim

[... normal data collection ...]

📝 Writing emissions to local file (dry run)...
✅ Emissions written to ./chilli_emissions/matts_famous_chilli_emissions.jsonl
   (12 ratings)

💡 To sync to ReSim, run without --dry-run flag
```

### Emission File Format

The local emissions file (`./chilli_emissions/<name>_emissions.jsonl`) contains NDJSON (newline-delimited JSON):

```json
{"$metadata": {"topic": "individual_rating"}, "$data": {"chilli_name": "Matt's Famous Chilli", "person_name": "Sarah", "visual": 4, "aroma": 3, "texture": 4, "taste": 5, "spice_level": 3, "overall_score": 19}}
{"$metadata": {"topic": "individual_rating"}, "$data": {"chilli_name": "Matt's Famous Chilli", "person_name": "Alex", "visual": 3, "aroma": 4, "texture": 3, "taste": 4, "spice_level": 4, "overall_score": 18}}
```

This format is compatible with ReSim's emission system and can be uploaded manually if needed.

## Data Structure

### Topics

#### `individual_rating`
Individual ratings from each person:
- `chilli_name` (string): Name of the chilli
- `person_name` (string): Rater's name
- `visual` (int): Visual rating (1-5)
- `aroma` (int): Aroma rating (1-5)
- `texture` (int): Texture rating (1-5)
- `taste` (int): Taste rating (1-5)
- `spice_level` (int): Spice level rating (1-5)
- `overall_score` (int): Sum of all categories (5-25)

Note: All aggregate statistics (averages, rankings, etc.) are calculated dynamically using SQL queries on the individual ratings.

## Metrics & Visualizations

The system provides comprehensive metrics at both test and batch levels:

### Test-Level Metrics (Per Chilli)

View these metrics on each individual chilli's test page:

1. **Individual Scores for This Chilli** (Table)
   - Shows the score given by each person for this specific chilli
   - Displays all 5 categories plus overall score per person

2. **Average Scores for This Chilli** (Table)
   - Average score for each category across all raters
   - Shows the chilli's performance profile

3. **Overall Average Score** (Scalar)
   - Single number showing this chilli's overall average score

### Batch-Level Metrics (All Chillies)

View these metrics on the batch page to compare all chillies:

#### Rankings & Winners

1. **Ranking by Person** (Table)
   - Shows each person's ranking of all chillies (1st, 2nd, 3rd, etc.)
   - Example: For "Matt" - Chilli 1 was 3rd, Chilli 2 was 1st, Chilli 3 was 2nd

2. **Winner by Average Placement** (Table) ⭐ **DETERMINES THE OVERALL WINNER**
   - Calculates the overall winner by average placement across all people
   - Lower average placement = better (1.0 is best)
   - This is the fairest way to determine the winner!

3. **Average Placement Chart** (Bar Chart)
   - Visual representation of average placement (lower bars are better)
   - Easy to see the winner at a glance

4. **Best Chilli Per Category** (Table)
   - Shows which chilli scored highest in each category by raw score
   - Identifies category champions (e.g., "Best Visual", "Best Taste")

#### Detailed Comparisons

5. **All Individual Ratings** (Table)
   - Complete view of all ratings from all people for all chillies
   - Sorted by person, then by score

6. **Average Ratings Comparison** (Table)
   - Compares average ratings across all chillies
   - Shows all 5 categories plus overall score
   - Sorted by overall score (highest first)

7. **Overall Score by Chilli** (Bar Chart)
   - Visual comparison of overall scores for all chillies

8. **Category Breakdown** (Line Chart)
   - Shows how each chilli performs across all 5 categories
   - Multiple lines allow easy comparison between chillies
   - Great for identifying strengths and weaknesses

## How the Winner is Determined

The system uses a **placement-based ranking system** which is more fair than raw scores:

### Example Scenario

Three chilli recipes are ranked by three people (Matt, Sarah, and Alex):

| Person | Matt's Chilli | Sarah's Chilli | Alex's Chilli |
|--------|---------------|----------------|---------------|
| Matt   | 16 (3rd)      | 23 (1st)       | 20 (2nd)      |
| Sarah  | 22 (1st)      | 18 (3rd)       | 20 (2nd)      |
| Alex   | 21 (2nd)      | 22 (1st)       | 19 (3rd)      |

**Average Placements:**
- Matt's Chilli: (3 + 1 + 2) / 3 = **2.0** (2nd place overall)
- Sarah's Chilli: (1 + 3 + 1) / 3 = **1.67** (1st place overall) 🏆
- Alex's Chilli: (2 + 2 + 3) / 3 = **2.33** (3rd place overall)

**Winner: Sarah's Chilli** with the lowest average placement!

The system tracks each person by name, making it easy to see individual preferences and compare rankings.

### Why This Method?

- **Fairer than raw scores**: Accounts for different people's rating scales
- **Considers all opinions equally**: Each person's ranking matters the same
- **Intuitive**: Lower placement number = better chilli
- **Handles ties gracefully**: SQL RANK() function manages tied scores

## Configuration

The metrics configuration is located at `.resim/metrics/config.yaml` and defines:
- Topic schemas for validation
- SQL queries for data analysis
- Visualization templates (table, bar, line, scalar)

## ReSim Integration

Data is automatically emitted to:
- **Project**: `chilli-ranking`
- **System**: `i-love-chilli`
- **Batch**: `October Offsite Ranking`
- **Test Suite**: `Ranktastic`

Each chilli recipe gets its own test within the batch, making it easy to compare multiple recipes.

### Workflow

1. **Run the script multiple times** - once for each chilli recipe you want to rank
2. **Each run creates a new test** - named after the chilli recipe
3. **Batch metrics aggregate results** - all metrics are batch-level, so they automatically combine data from all recipes
4. **View in ReSim dashboard** - see comprehensive comparisons across all recipes in the batch

## Tips

- **Be Consistent**: Use the same scale interpretation across all raters
- **Multiple Sessions**: Run the script multiple times for different chilli recipes
- **Compare Results**: Use the ReSim dashboard to compare recipes side-by-side
- **Track Over Time**: Re-rank recipes to see if opinions change (or if someone improved their recipe!)
- **Blind Testing**: Consider blind tastings where raters don't know whose chilli they're rating
- **Fair Play**: The script automatically prevents people from rating their own chilli - this ensures fair competition!
- **Minimum Raters**: Each chilli needs at least one rating (excluding the maker) to be valid

## Requirements

- Python 3.x
- ReSim SDK (`resim` package)
- Proper ReSim authentication (via environment variables or device code flow)
