@echo off
setlocal enabledelayedexpansion

rem Copy the skill folders in this directory into the user-level skill directories
rem of Codex, Claude Code and Copilot, replacing the old copies.

set "SRC=%~dp0"
set "SKILL_COUNT=0"
set "TARGET_COUNT=0"
set "COPY_COUNT=0"

rem Count source skills: subdirectories that contain a SKILL.md file
for /d %%S in ("%SRC%*") do (
    if exist "%%S\SKILL.md" set /a SKILL_COUNT+=1
)

if %SKILL_COUNT%==0 (
    echo No skill found in "%SRC%". A skill folder must contain SKILL.md.
    goto :fail
)

rem Copy every skill into every target root
for %%R in (".codex\skills" ".claude\skills" ".copilot\skills") do (
    set "ROOT=%USERPROFILE%\%%~R"
    set /a TARGET_COUNT+=1

    if not exist "!ROOT!" (
        mkdir "!ROOT!"
        echo Created !ROOT!
    )

    for /d %%S in ("%SRC%*") do (
        if exist "%%S\SKILL.md" (
            set "DEST=!ROOT!\%%~nxS"
            rem Delete the old copy first so files removed from the source do not survive
            if exist "!DEST!" rmdir /s /q "!DEST!"
            robocopy "%%S" "!DEST!" /E /NFL /NDL /NJH /NJS /NP >nul
            set "RB_ERR=!errorlevel!"
            if !RB_ERR! GEQ 8 (
                echo FAILED: %%~nxS -^> !DEST!  ^(robocopy exit code !RB_ERR!^)
                goto :fail
            )
            set /a COPY_COUNT+=1
            echo Copied %%~nxS -^> !DEST!
        )
    )
)

rem Print the summary
echo.
echo Done. %SKILL_COUNT% skill(s) x %TARGET_COUNT% target(s) = %COPY_COUNT% copies.
goto :end

:fail
echo.
echo Distribution failed. Nothing else was copied.

:end
pause
