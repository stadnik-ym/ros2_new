#!/bin/bash
echo "видаляємо build, install, log..."
rm -rf build/ install/ log/
echo "збираємо проект..."
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if [ $? -eq 0 ]; then
    echo "проект було успішно зібрано"
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo "оточення було успішно запущено!"
    else
        echo "файл setup.bash в директорії install/ не знайдено"
    fi
else
    echo "збірка завершилась з помилкою"
fi
