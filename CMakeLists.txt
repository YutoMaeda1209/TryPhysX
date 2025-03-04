cmake_minimum_required(VERSION 3.10)

# プロジェクト名とバージョン
project(MyProject VERSION 1.0)

# 実行ファイルを生成する
add_executable(Hello src/helloworld.cpp)

target_include_directories(include)