-- user name: debugcrew
-- pw: 1234

use home_ai;

drop table if exists history;
drop table if exists schedules;
drop table if exists users;
drop table if exists robots;
drop table if exists robot_types;
drop table if exists detections;
drop table if exists items;
drop table if exists commands;
drop table if exists positions;
drop table if exists postures;

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

insert into items values ('i2602070001', 'choco stick', 10, 3);

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
        w         float not null
    );

insert into positions values ('p2602070001', 'drop zone', 0.2269, 0.2037, 0.7076);
insert into positions values ('p2602100001', 'charger', 0.0, 0.0, 0.0);
insert into positions values ('p2602100002', 'living room', 4.0, 5.0, 6.0);
insert into positions values ('p2602100003', 'bed room', 7.0, 8.0, 9.0);

create table postures (
        pos_id   varchar(11) primary key,
        pos_name varchar(30) not null,
        j1       float not null,
        j2       float not null,
        j3       float not null,
        j4       float not null,
        j5       float not null,
        j6       float not null,
        gap      int not null
    );

-- "p" + 6 digit date + 4 digit int
insert into postures
values
  ( "p2602050001", "pinky_side", 0, 40, -90, 5, 4, 135, 0 ),
  ( "p2602050002", "shelve_side", 90, 40, -90, 5, 4, 135, 0 ),
  ( "p2602050003", "trash_side", -90, 40, -90, 5, 4, 135, 0 );

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

insert into schedules values ('s2602070001', 'c2602070001', 'i2602070001', 'p2602070001', '15:40:00', 1, True);

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
