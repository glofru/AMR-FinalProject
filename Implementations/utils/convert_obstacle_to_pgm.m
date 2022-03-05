function convert_obstacle_to_pgm(map, filename)
    file = fopen(filename, 'w');

    fprintf(file, ['P2\n' num2str(map.col) ' ' num2str(map.row) '\n' '3\n']);

    for i=1:map.row
        for j=1:map.col
            if map.isObstacle(i, j)
                fprintf(file, '0 ');
            else
                fprintf(file, '3 ');
            end
        end
        fprintf(file, '\n');
    end

    fclose(file);
end