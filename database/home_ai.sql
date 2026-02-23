-- user name: debugcrew
-- pw: 1234

use home_ai;

drop table if exists history;
drop table if exists schedules;
drop table if exists users;
drop table if exists robots;
drop table if exists robot_types;
drop table if exists detections;
drop table if exists angles;
drop table if exists items;
drop table if exists commands;
drop table if exists positions;

create table users (
        user_id   varchar(30) primary key,
        user_pw   varchar(60) not null,
        user_name varchar(30) not null
    );

insert into users
values
    ('asdf', 'asdf', 'foobar');

create table robot_types (
    robot_type varchar(30) primary key
);

insert into robot_types (robot_type)
values
    ('transport'),
    ('cleaner'),
    ('arm');

create table robots (
        robot_id   varchar(11) primary key,
        namespace  varchar(20) not null,
        robot_type varchar(30) not null,
        robot_name varchar(30) not null,
        foreign key (robot_type) references robot_types (robot_type)
    );

create table detections (
        detection_type varchar(11) primary key,
        confidence     float default 0.0,
        is_reported    bool default false,
        img_path       varchar(100)
    );

create table items (
        item_id   varchar(11) primary key,
        item_name varchar(30) not null,
        amount    int default 0,
        frequency int default 0
    );

insert into items
values
    ('i2602150001', 'choco', 10, 3),
    ('i2602150002', 'juice', 10, 3),
    ('i2602150003', 'pill', 10, 3);

create table angles (
        angle_id   varchar(11) primary key,
        item_id  varchar(11),
        angle_name varchar(30) not null,
        j1       float not null,
        j2       float not null,
        j3       float not null,
        j4       float not null,
        j5       float not null,
        j6       float not null,
        foreign key (item_id) references items (item_id)
    );

-- "a" + 6 digit date + 4 digit int
insert into angles
values
    ('a2602150001', null, 'home', 1.66, -0.08, 3.07, -9.14, 16.69, -42.97),
    ('a2602150002', null, 'shelve side', 92.63, 14.76, 3.25, -77.51, 18.8, -43.33),
    ('a2602150003', 'i2602150001', 'pick choco', 106.43, -69.08, -19.07, -8.7, 13.18, -33.57),
    ('a2602150004', 'i2602150002', 'pick juice', 99.66, -59.23, -19.07, -16.08, 8.78, -37.17),
    ('a2602150005', 'i2602150003', 'pick pill', 131.66, -68.55, -19.68, 0.61, 13.79, -10.54),
    ('a2602150006', null, 'pinky side', 0.96, -2.1, -6.76, -68.9, 19.33, -46.66),
    ('a2602150007', null, 'drop', 12.56, -76.81, -59.94, 46.66, 12.74, 53.43),
    ('a2602150008', null, 'trash side', -88.85, 0.26, -33.48, -13.35, 20.47, -45.7),
    ('a2602150009', null, 'trash general', -63.1, -61.96, -6.85, 13.44, 3.69, -27.07),
    ('a2602150010', null, 'trash paper', -86.3, -64.16, -7.38, 20.03, 8.87, -34.27);

create table commands (
        cmd_id   varchar(11) primary key,
        cmd_type varchar(30) not null
    );

insert into commands values ('c2602070001', 'fetch');
insert into commands values ('c2602100002', 'take');

create table positions (
        position_id   varchar(11) primary key,
        position_name varchar(30) not null,
        x             float not null,
        y             float not null,
        yaw           float not null
    );

insert into positions values ('p2602150001', 'charger', 0.0, 0.05, 0.0);
insert into positions values ('p2602150002', 'drop zone', 0.24, 0.0, 90.0);
insert into positions values ('p2602150003', 'living room', 0.55, 0.95, -90.0);
insert into positions values ('p2602150004', 'bed room', 0.0, 0.9, -90.0);

create table schedules (
        schedule_id  varchar(11) primary key,
        cmd_id       varchar(11),
        item_id      varchar(11),
        position_id  varchar(11),
        execute_time time not null,
        cycle        int default 1,
        on_weekends  bool default false,
        foreign key (cmd_id)      references commands (cmd_id),
        foreign key (item_id)     references items (item_id),
        foreign key (position_id) references positions (position_id)
    );

insert into schedules
values
    ('s2602070001', 'c2602070001', 'i2602150001', 'p2602150004', '15:40:00', 1, True);

create table history (
        history_id     varchar(11) primary key,
        user_id        varchar(30),
        item_id        varchar(11),
        robot_1        varchar(11),
        robot_2        varchar(11),
        position_id    varchar(11),
        cmd_id         varchar(11),
        schedule_id    varchar(11),
        detection_type varchar(11),
        time_start     time not null,
        time_end       time not null,
        is_successful  bool default false,
        foreign key (user_id)        references users (user_id),
        foreign key (item_id)        references items (item_id),
        foreign key (robot_1)        references robots (robot_id),
        foreign key (robot_2)        references robots (robot_id),
        foreign key (position_id)    references positions (position_id),
        foreign key (cmd_id)         references commands (cmd_id),
        foreign key (schedule_id)    references schedules (schedule_id),
        foreign key (detection_type) references detections (detection_type)
    );

commit;
