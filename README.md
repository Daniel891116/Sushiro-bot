# Sushiro-bot

## Development
1. Make your local main up-to-date

    ```sh
    git fetch --all
    ```
2. Commit your Changes

    ```sh
    git add --all
    git commit -m "{your commit message}"
    ```

3. Sync your local branch with upstream/main

    ```sh
    git rebase origin/main
    ```

4. If conflict aroused, goto VScode and apply changes first

    ```sh
    git add --all
    git rebase --continue
    ```
    - continue step 4 unitl no conflicts exist

4. Push to remote
   ```sh
   git push origin main
   ```