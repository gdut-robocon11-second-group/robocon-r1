Get-ChildItem -Recurse -Include *.cpp,*.hpp,*.cxx,*.cc,*.ino |
Where-Object {
    $_.FullName -notmatch '\\build\\' -and
    $_.FullName -notmatch '\\third_party\\' -and
    $_.FullName -notmatch '\\external\\' -and
    $_.FullName -notmatch '\\vendor\\' -and
    $_.FullName -notmatch '\\generated\\' -and
    $_.FullName -notmatch '\\out\\' -and
    $_.FullName -notmatch '\\cmake\\' -and
    $_.FullName -notmatch '\\Drivers\\' -and
    $_.FullName -notmatch '\\Lib\\' -and
    $_.FullName -notmatch '\\Middlewares\\'
} | ForEach-Object {
    clang-format -i $_.FullName
}
