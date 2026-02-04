-- db name: debugcrew
-- user name: debugcrew
-- pw: debug_crew_1234

use debugcrew;

drop table if exists logs;
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

insert into users (user_id, user_pw, user_name) values ('asdf', 'asdf', 'foobar');

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

create table commands (
        cmd_id   varchar(11) primary key,
        cmd_type varchar(30) not null
    );

create table positions (
        position_id   varchar(11) primary key,
        position_name varchar(30) not null,
        x             float not null,
        y             float not null,
        theta         float not null
    );

create table postures (
        pos_id   varchar(11) primary key,
        pos_name varchar(30) not null,
        j1       float not null,
        j2       float not null,
        j3       float not null,
        j4       float not null,
        j5       float not null,
        j6       float not null,
        angle    float not null,
        gap      int not null
    );

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

create table logs (
        log_id         varchar(11) primary key,
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
