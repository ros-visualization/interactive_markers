#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
#include <tf/tf.h>

#include "marker_shmup/HeroCommand.h"

#include <cstdlib>
#include <ctime>

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using namespace marker_shmup;

ros::Publisher g_marker_pub;
ros::Publisher g_marker_array_pub;
ros::Subscriber g_command_sub;
ros::Timer g_update_timer;
HeroCommandConstPtr g_cmd;

MarkerArray g_markers;
typedef std::vector<uint32_t> V_u32;
V_u32 g_marker_free_list;

uint32_t g_id_counter = 0;

template<typename T>
void setXYZ(T& t, double x, double y, double z)
{
  t.x = x;
  t.y = y;
  t.z = z;
}

template<typename T>
void setXYZW(T& t, double x, double y, double z, double w)
{
  t.x = x;
  t.y = y;
  t.z = z;
  t.w = w;
}

template<typename T>
void updateXYFromVel(T& t, double dt, double vx, double vy)
{
  t.x += vx*dt;
  t.y += vy*dt;
}

void updateXYFromAccel(float& x, float& y, double dt, double ax, double ay)
{
  x += ax * dt;
  y += ay * dt;
}

template<typename C>
void setColor(C& c, float r, float g, float b, float a = 1.0)
{
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
}

template<typename T>
bool pointInCircle(const T& circle_origin, float radius, const T& point)
{
  float dx = circle_origin.x - point.x;
  float dy = circle_origin.y - point.y;

  float dist_squared = dx*dx + dy*dy;
  return dist_squared < radius*radius;
}

uint32_t allocateId()
{
  return ++g_id_counter;
}

void initMarker(Marker& m)
{
  m.ns = "shmup";
  m.id = allocateId();
  m.header.frame_id = "/shmup";
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
}

uint32_t allocatePersistentMarker()
{
  if (g_marker_free_list.empty())
  {
    uint32_t old_size = g_markers.markers.size();
    uint32_t new_size = old_size * 2;
    if (g_markers.markers.empty())
    {
      new_size = 100;
    }

    g_markers.markers.resize(new_size);
    uint32_t i = old_size;
    MarkerArray::_markers_type::iterator it = g_markers.markers.begin() + old_size;
    MarkerArray::_markers_type::iterator end = g_markers.markers.end();
    for (; it != end; ++it)
    {
      Marker& m = *it;
      initMarker(m);
      m.action = Marker::DELETE;
      m.color.a = 0.0;
      g_marker_free_list.push_back(i);
      ++i;
    }
  }

  uint32_t handle = g_marker_free_list.back();
  g_marker_free_list.pop_back();
  Marker& m = g_markers.markers[handle];
  m.action = Marker::ADD;
  m.color.a = 1.0;
  setXYZ(m.pose.position, 0.0, 0.0, 0.0);
  setXYZW(m.pose.orientation, 0.0, 0.0, 0.0, 1.0);
  m.points.clear();
  return handle;
}

void deallocatePersistentMarker(uint32_t handle)
{
  g_marker_free_list.push_back(handle);
  Marker& m = g_markers.markers[handle];
  m.action = Marker::DELETE;
}

Marker& resolvePersistentMarker(uint32_t handle)
{
  return g_markers.markers[handle];
}

enum Owner
{
  HERO,
  ENEMY
};

typedef std::vector<std_msgs::ColorRGBA> V_Color;
typedef std::vector<ros::Duration> V_Duration;

template<typename T>
T distanceSquared(T x1, T y1, T x2, T y2)
{
  T dx = x2 - x1;
  T dy = y2 - y1;
  return (dx * dx) + (dy * dy);
}

struct Particle
{
  float x;
  float y;
  float vx;
  float vy;
  float deathdist;
};
typedef std::vector<Particle> V_Particle;

struct Explosion
{
  Explosion()
  : marker_handle(0)
  , vrad(0.0)
  , done(false)
  , in_use(false)
  {
  }

  void explode(float x, float y, const std_msgs::ColorRGBA& color, ros::Duration dur)
  {
    marker_handle = allocatePersistentMarker();
    explosion_start = ros::Time::now();

    Marker& m = resolvePersistentMarker(marker_handle);
    m.type = Marker::POINTS;
    m.color = color;
    m.color.a = 0.3;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    setXYZ(m.pose.position, x, y, 0.0);

    total_dur = dur;

    this->x = x;
    this->y = y;
    done = false;
  }

  bool isDone()
  {
    return done;
  }

  void spawnParticle()
  {
    Particle p;
    p.x = 0.0;
    p.y = 0.0;

    float vel = (rand() % 100000) / 10000.0;
    p.vx = ((rand() % 100000) / 100000.0) * vel;
    p.vy = vel - p.vx;
    if (rand() % 2 == 0)
      p.vx = -p.vx;
    if (rand() % 2 == 0)
      p.vy = -p.vy;

    p.deathdist = (rand() % 40 + 1) * total_dur.toSec();
    particles.push_back(p);
  }

  void update(double dt)
  {
    if (isDone())
    {
      return;
    }

    ros::Time now = ros::Time::now();

    bool time_up = false;
    if (now - explosion_start < total_dur)
    {
      uint32_t to_spawn = (rand() % 100) * (1.0 - ((now - explosion_start).toSec() / total_dur.toSec()));
      for (uint32_t i = 0; i < to_spawn; ++i)
      {
        spawnParticle();
      }
    }
    else
    {
      time_up = true;
    }

    V_Particle::iterator it = particles.begin();
    V_Particle::iterator end = particles.end();
    for (; it != end;)
    {
      Particle& p = *it;
      float distsq = distanceSquared(x, y, x + p.x, y + p.y);
      if (distsq >= p.deathdist * p.deathdist)
      {
        std::swap(particles.back(), *it);

        if (particles.size() > 1)
        {
          particles.resize(particles.size() - 1);
        }
        else
        {
          particles.clear();
          break;
        }
      }
      else
      {
        updateXYFromVel(p, dt, p.vx, p.vy);

        ++it;
      }
    }

    if (time_up && particles.empty())
    {
      done = true;
      deallocatePersistentMarker(marker_handle);
    }
    else if (particles.empty())
    {
      Marker& m = resolvePersistentMarker(marker_handle);
      m.points.clear();
    }
    else
    {
      Marker& m = resolvePersistentMarker(marker_handle);
      m.points.resize(particles.size());
      uint32_t i = 0;
      it = particles.begin();
      end = particles.end();
      for (; it != end; ++i, ++it)
      {
        Particle& p = *it;
        setXYZ(m.points[i], p.x, p.y, 0.0);
      }

      if (time_up)
      {
        m.color.a -= dt * 0.1;

        if (m.color.a <= 0.0)
        {
          done = true;
          deallocatePersistentMarker(marker_handle);
        }
      }
    }

  }

  uint32_t marker_handle;
  ros::Duration total_dur;
  ros::Time explosion_start;
  float x;
  float y;
  float vrad;
  bool done;
  bool in_use;

  V_Particle particles;
};

typedef std::vector<Explosion> V_Explosion;
V_Explosion g_explosions;

Explosion& allocateExplosion()
{
  V_Explosion::iterator it = g_explosions.begin();
  V_Explosion::iterator end = g_explosions.end();
  for (; it != end; ++it)
  {
    Explosion& e = *it;
    if (!e.in_use)
    {
      e = Explosion();
      e.in_use = true;
      return e;
    }
  }

  size_t old_size = g_explosions.size();
  size_t new_size = old_size * 2;
  if (new_size == 0)
  {
    new_size = 16;
  }
  g_explosions.resize(new_size);
  Explosion& e = g_explosions[old_size];
  e.in_use = true;
  return e;
}

void updateExplosions(double dt)
{
  V_Explosion::iterator it = g_explosions.begin();
  V_Explosion::iterator end = g_explosions.end();
  for (; it != end; ++it)
  {
    Explosion& e = *it;
    if (e.in_use)
    {
      e.update(dt);
      if (e.isDone())
      {
        e.in_use = false;
      }
    }
  }
}

struct Projectile
{
  Projectile()
  : marker_handle(0)
  , vx(0.0)
  , vy(0.0)
  , owner(ENEMY)
  , in_use(false)
  {}

  uint32_t marker_handle;
  float vx;
  float vy;
  Owner owner;
  bool in_use;
};
typedef std::vector<Projectile> V_Projectile;
V_Projectile g_projectiles;

Projectile& allocateProjectile()
{
  V_Projectile::iterator it = g_projectiles.begin();
  V_Projectile::iterator end = g_projectiles.end();
  for (; it != end; ++it)
  {
    Projectile& p = *it;
    if (!p.in_use)
    {
      p = Projectile();
      p.marker_handle = allocatePersistentMarker();
      p.in_use = true;

      Marker& m = resolvePersistentMarker(p.marker_handle);
      m.type = Marker::LINE_LIST;
      m.points.resize(8);
      setXYZ(m.points[0], -0.25, 0.0, 0.0);
      setXYZ(m.points[1], 0.25, 0.0, 0.0);
      setXYZ(m.points[2], 0.0, -0.25, 0.0);
      setXYZ(m.points[3], 0.0, 0.25, 0.0);
      setXYZ(m.points[4], -0.2, -0.2, 0.0);
      setXYZ(m.points[5], 0.2, 0.2, 0.0);
      setXYZ(m.points[6], -0.2, 0.2, 0.0);
      setXYZ(m.points[7], 0.2, -0.2, 0.0);
      return p;
    }
  }

  size_t old_size = g_projectiles.size();
  size_t new_size = old_size * 2;
  if (new_size == 0)
  {
    new_size = 16;
  }
  g_projectiles.resize(new_size);
  Projectile& p = g_projectiles[old_size];
  p.marker_handle = allocatePersistentMarker();
  p.in_use = true;
  Marker& m = resolvePersistentMarker(p.marker_handle);
  m.type = Marker::LINE_LIST;
  m.points.resize(8);
  setXYZ(m.points[0], -0.25, 0.0, 0.0);
  setXYZ(m.points[1], 0.25, 0.0, 0.0);
  setXYZ(m.points[2], 0.0, -0.25, 0.0);
  setXYZ(m.points[3], 0.0, 0.25, 0.0);
  setXYZ(m.points[4], -0.2, -0.2, 0.0);
  setXYZ(m.points[5], 0.2, 0.2, 0.0);
  setXYZ(m.points[6], -0.2, 0.2, 0.0);
  setXYZ(m.points[7], 0.2, -0.2, 0.0);
  return p;
}

uint32_t g_playing_field_handle = 0;
void initPlayingField()
{
  g_playing_field_handle = allocatePersistentMarker();
  Marker& m = resolvePersistentMarker(g_playing_field_handle);

  m.type = Marker::LINE_STRIP;
  setColor(m.color, 0.2, 0.8, 0.2);
  m.scale.x = 0.2;
  m.points.resize(5);
  setXYZ(m.points[0], -1.0, -10.0, 0.0);
  setXYZ(m.points[1], -1.0, 10.0, 0.0);
  setXYZ(m.points[2], 20.0, 10.0, 0.0);
  setXYZ(m.points[3], 20.0, -10.0, 0.0);
  setXYZ(m.points[4], -1.0, -10.0, 0.0);
}

static const ros::Duration HERO_PROJECTILE_FIRE_DELAY(0.4);
static const float HERO_MAX_SPEED = 5.0;
static const ros::Duration HERO_SPAWN_DURATION(3.0);
static const ros::Duration HERO_EXPLODE_DURATION(2.0);

struct Hero
{
  Hero()
  : finished_spawning(false)
  , exploding(false)
  {}

  uint32_t marker_handle;
  uint32_t shield_handle;
  ros::Time last_fire_time;
  ros::Time spawn_time;
  ros::Time explode_time;
  bool finished_spawning;
  bool exploding;

  void spawn()
  {
    finished_spawning = false;
    exploding = false;
    spawn_time = ros::Time::now();
    Marker& m = resolvePersistentMarker(marker_handle);
    setXYZ(m.pose.position, 0.0, 0.0, 0.0);
    m.color.a = 0.0;
  }

  void fire()
  {
    if (ros::Time::now() - last_fire_time < HERO_PROJECTILE_FIRE_DELAY)
    {
      return;
    }

    last_fire_time = ros::Time::now();

    Projectile& p = allocateProjectile();
    p.vx = 3.0;
    p.owner = HERO;

    Marker& my_m = resolvePersistentMarker(marker_handle);
    Marker& m = resolvePersistentMarker(p.marker_handle);
    setXYZ(m.pose.position, my_m.pose.position.x + 1.0, my_m.pose.position.y, 0.0);
    m.scale.x = 0.05;
    setColor(m.color, 0.2, 0.3, 0.9, 0.5);
  }

  void update(double dt)
  {
    Marker& m = resolvePersistentMarker(marker_handle);

    if (exploding)
    {
      ros::Duration d = ros::Time::now() - explode_time;
      if (d >= HERO_EXPLODE_DURATION)
      {
        exploding = false;
        spawn();
      }
      else
      {
        return;
      }
    }

    if (!finished_spawning)
    {
      ros::Duration d = ros::Time::now() - spawn_time;
      if (d >= HERO_SPAWN_DURATION)
      {
        finished_spawning = true;
        m.color.a = 1.0;
      }
      else
      {
        double frac = d.toSec() / HERO_SPAWN_DURATION.toSec();
        m.color.a = frac;
        setXYZ(m.scale, frac*0.5, frac*0.5, frac*0.5);
        return;
      }
    }

    if (g_cmd->fire && !g_cmd->shield)
    {
      fire();
    }

    m.pose.position.y += std::max(-HERO_MAX_SPEED, std::min(HERO_MAX_SPEED, g_cmd->y)) * dt;
    m.pose.position.x += std::max(-HERO_MAX_SPEED, std::min(HERO_MAX_SPEED, g_cmd->x)) * dt;

    m.pose.position.x = std::max(0.0, std::min(18.0, m.pose.position.x));
    m.pose.position.y = std::max(-9.0, std::min(9.0, m.pose.position.y));

    Marker& sm = resolvePersistentMarker(shield_handle);
    sm.pose = m.pose;
    sm.pose.position.z += 0.1;
    sm.pose.position.x += 0.25;

    if (g_cmd->shield)
    {
      sm.color.a = 0.5;
    }
    else
    {
      sm.color.a = 0.0;
    }
  }

  void explode()
  {
    finished_spawning = false;
    exploding = true;
    explode_time = ros::Time::now();
    Explosion& e = allocateExplosion();
    Marker& m = resolvePersistentMarker(marker_handle);
    e.explode(m.pose.position.x + 0.5, m.pose.position.y, m.color, HERO_EXPLODE_DURATION);

    m.color.a = 0.0;
  }

  bool checkCollision(Projectile& p)
  {
    Marker& p_m = resolvePersistentMarker(p.marker_handle);
    Marker& my_m = resolvePersistentMarker(marker_handle);
    Marker::_pose_type::_position_type center = my_m.pose.position;
    center.x += 0.5;
    return pointInCircle(center, 1.0, p_m.pose.position);
  }
};
Hero g_hero;

void initHero()
{
  g_hero.marker_handle = allocatePersistentMarker();
  Marker& hero_marker = resolvePersistentMarker(g_hero.marker_handle);
  hero_marker.type = Marker::CUBE_LIST;
  setXYZ(hero_marker.scale, 0.5, 0.5, 0.5);
  setColor(hero_marker.color, 0.3, 0.8, 0.8);
  hero_marker.points.resize(5);
  setXYZ(hero_marker.points[0], 0.0, 0.0, 0.0);
  setXYZ(hero_marker.points[1], 0.0, 0.5, 0.0);
  setXYZ(hero_marker.points[2], 0.0, -0.5, 0.0);
  setXYZ(hero_marker.points[3], 0.5, 0.0, 0.0);
  setXYZ(hero_marker.points[4], 1.0, 0.0, 0.0);

  g_hero.shield_handle = allocatePersistentMarker();
  Marker& shield_marker = resolvePersistentMarker(g_hero.shield_handle);
  shield_marker.type = Marker::SPHERE;
  setColor(shield_marker.color, 0.2, 0.8, 0.2, 0.0);
  setXYZ(shield_marker.scale, 3.0, 2.0, 1.0);

  g_hero.spawn();
}

const static float ENEMY_MAX_SPEED = 2.0;
const static float ENEMY_FADE_IN_TIME = 2.0;
static const ros::Duration ENEMY_PROJECTILE_FIRE_DELAY(0.7);
static const ros::Duration ENEMY_EXPLODE_DURATION(0.5);

struct Enemy
{
  Enemy()
  : marker_handle(0)
  , target_marker_handle(0)
  , in_use(false)
  , vx(0.0)
  , vy(0.0)
  {}

  uint32_t marker_handle;
  uint32_t target_marker_handle;
  ros::Time last_fire_time;
  ros::Time spawn_time;
  bool finished_spawning;
  bool in_use;
  float vx;
  float vy;
  float ax;
  float ay;

  float targetx;
  float targety;

  void spawn()
  {
    spawn_time = ros::Time::now();
    finished_spawning = false;

    marker_handle = allocatePersistentMarker();
    Marker& m = resolvePersistentMarker(marker_handle);
    m.type = Marker::CUBE_LIST;
    setXYZ(m.pose.position, 19.0, 0.0, 0.0);
    setXYZ(m.scale, 0.3, 0.3, 0.3);
    setColor(m.color, 0.8, 0.3, 0.3);
    m.points.resize(5);
    setXYZ(m.points[0], 0.0, 0.0, 0.0);
    setXYZ(m.points[1], 0.0, 0.3, 0.0);
    setXYZ(m.points[2], 0.0, -0.3, 0.0);
    setXYZ(m.points[3], -0.3, 0.0, 0.0);
    setXYZ(m.points[4], -0.6, 0.0, 0.0);
    m.color.a = 0.0;

    vx = 0.0;
    vy = 0.0;

    randomTarget();
    updateAccel();

    m.pose.position.y = rand() % 18 - 10;

#if 0
    target_marker_handle = allocatePersistentMarker();
#endif
  }

  void destroy()
  {
    deallocatePersistentMarker(marker_handle);
    marker_handle = 0;

    if (target_marker_handle)
    {
      deallocatePersistentMarker(target_marker_handle);
      target_marker_handle = 0;
    }
  }

  void explode()
  {
    Explosion& e = allocateExplosion();
    Marker& m = resolvePersistentMarker(marker_handle);
    e.explode(m.pose.position.x, m.pose.position.y, m.color, ros::Duration(ENEMY_EXPLODE_DURATION));
    destroy();
  }

  bool checkCollision(Projectile& p)
  {
    Marker& p_m = resolvePersistentMarker(p.marker_handle);
    Marker& my_m = resolvePersistentMarker(marker_handle);
    Marker::_pose_type::_position_type center = my_m.pose.position;
    center.x -= 0.3;
    return pointInCircle(center, 0.6, p_m.pose.position);
  }

  void updateAccel()
  {
    Marker& m = resolvePersistentMarker(marker_handle);
    ax = ((targetx - m.pose.position.x) < 0 ? -1 : 1) * 2.0;
    ay = ((targety - m.pose.position.y) < 0 ? -1 : 1) * 2.0;
  }

  void updatePositionAndVelocity(double dt)
  {
    Marker& m = resolvePersistentMarker(marker_handle);
    updateXYFromAccel(vx, vy, dt, ax, ay);
    vx = std::max(std::min(vx, ENEMY_MAX_SPEED), -ENEMY_MAX_SPEED);
    vy = std::max(std::min(vy, ENEMY_MAX_SPEED), -ENEMY_MAX_SPEED);
    updateXYFromVel(m.pose.position, dt, vx, vy);
  }

  void randomTarget()
  {
    targetx = rand() % 40 - 21;
    targety = rand() % 18 - 9;

    if (target_marker_handle)
    {
      Marker& tm = resolvePersistentMarker(target_marker_handle);
      tm.type = Marker::SPHERE;
      setXYZ(tm.pose.position, targetx, targety, 0);
      setXYZ(tm.scale, 0.3, 0.3, 0.3);
      setColor(tm.color, 0.0, 1.0, 0.0);
    }
  }

  bool atTarget()
  {
    Marker& m = resolvePersistentMarker(marker_handle);
    float dx = fabsf(m.pose.position.x - targetx);
    float dy = fabsf(m.pose.position.y - targety);
    return dx <= 1.0 || dy <= 1.0;
  }

  void fire()
  {
    if (ros::Time::now() - last_fire_time < ENEMY_PROJECTILE_FIRE_DELAY)
    {
      return;
    }

    last_fire_time = ros::Time::now();

    Projectile& p = allocateProjectile();
    p.vx = -3.0;
    p.owner = ENEMY;

    Marker& my_m = resolvePersistentMarker(marker_handle);
    Marker& m = resolvePersistentMarker(p.marker_handle);
    setXYZ(m.pose.position, my_m.pose.position.x - 1.0, my_m.pose.position.y, 0.0);
    m.scale.x = 0.05;
    setColor(m.color, 0.9, 0.1, 0.1, 0.5);
  }
};
typedef std::vector<Enemy> V_Enemy;
V_Enemy g_enemies;
V_u32 g_enemy_free_list;
uint32_t g_enemy_count_target = 3;
uint32_t g_enemy_count = 0;

uint32_t allocateEnemy()
{
  if (g_enemy_free_list.empty())
  {
    uint32_t old_size = g_enemies.size();
    uint32_t new_size = old_size * 2;
    if (g_enemies.empty())
    {
      new_size = 10;
    }

    g_enemies.resize(new_size);
    uint32_t i = old_size;
    V_Enemy::iterator it = g_enemies.begin() + old_size;
    V_Enemy::iterator end = g_enemies.end();
    for (; it != end; ++it)
    {
      g_enemy_free_list.push_back(i);
      ++i;
    }
  }

  uint32_t handle = g_enemy_free_list.back();
  g_enemy_free_list.pop_back();
  Enemy& e = g_enemies[handle];
  e.in_use = true;
  ++g_enemy_count;
  return handle;
}

void deallocateEnemy(uint32_t handle)
{
  g_enemy_free_list.push_back(handle);
  Enemy& e = g_enemies[handle];
  e.in_use = false;
  --g_enemy_count;
}

Enemy& resolveEnemy(uint32_t handle)
{
  return g_enemies[handle];
}

void initEnemies()
{

}

void updateEnemies(double dt)
{
  V_Enemy::iterator it = g_enemies.begin();
  V_Enemy::iterator end = g_enemies.end();
  for (; it != end; ++it)
  {
    Enemy& e = *it;
    if (e.in_use)
    {
      if (e.finished_spawning)
      {
        Marker& m = resolvePersistentMarker(e.marker_handle);
        e.updateAccel();
        e.updatePositionAndVelocity(dt);

        if (m.pose.position.x > 20 || m.pose.position.x < -1.0)
        {
          e.destroy();
          deallocateEnemy(&e - &g_enemies.front());
        }
        else
        {
          if (e.atTarget())
          {
            e.randomTarget();
          }
          e.fire();
        }
      }
      else
      {
        Marker& m = resolvePersistentMarker(e.marker_handle);
        ros::Duration d = ros::Time::now() - e.spawn_time;
        if (d >= ros::Duration(2))
        {
          m.color.a = 1.0;
          e.finished_spawning = true;
        }
        else
        {
          m.color.a = d.toSec() / ENEMY_FADE_IN_TIME;
        }
      }
    }
  }

  if (g_enemy_count < g_enemy_count_target)
  {
    uint32_t handle = allocateEnemy();
    Enemy& e = resolveEnemy(handle);

    e.spawn();
  }
}

void updateProjectiles(double dt)
{
  V_Projectile::iterator it = g_projectiles.begin();
  V_Projectile::iterator end = g_projectiles.end();
  for (; it != end; ++it)
  {
    Projectile& p = *it;
    if (p.in_use)
    {
      Marker& m = resolvePersistentMarker(p.marker_handle);
      updateXYFromVel(m.pose.position, dt, p.vx, p.vy);
      tf::Quaternion quat;
      tf::quaternionMsgToTF(m.pose.orientation, quat);
      quat = quat * tf::createQuaternionFromYaw(6.0 * dt);
      tf::quaternionTFToMsg(quat, m.pose.orientation);

      if (m.pose.position.x > 20 || m.pose.position.x < -1.0)
      {
        deallocatePersistentMarker(p.marker_handle);
        p.in_use = false;
      }
      else
      {
        // If this is a hero projectile, check against enemies
        if (p.owner == HERO)
        {
          V_Enemy::iterator it = g_enemies.begin();
          V_Enemy::iterator end = g_enemies.end();
          for (; it != end; ++it)
          {
            Enemy& e = *it;
            if (e.finished_spawning && e.checkCollision(p))
            {
              {
                Explosion& pe = allocateExplosion();
                Marker& m = resolvePersistentMarker(p.marker_handle);
                pe.explode(m.pose.position.x, m.pose.position.y, m.color, ros::Duration(0.2));
              }

              e.explode();
              deallocateEnemy(&e - &g_enemies.front());
              deallocatePersistentMarker(p.marker_handle);
              p.in_use = false;


            }
          }
        }
        else // enemy owner
        {
          if (g_hero.finished_spawning && g_hero.checkCollision(p))
          {
            {
              Explosion& pe = allocateExplosion();
              Marker& m = resolvePersistentMarker(p.marker_handle);
              pe.explode(m.pose.position.x, m.pose.position.y, m.color, ros::Duration(0.2));
            }

            deallocatePersistentMarker(p.marker_handle);
            p.in_use = false;

            if (!g_cmd->shield)
            {
              g_hero.explode();
            }
          }
        }
      }
    }
  }
}

bool g_first = true;

void update(const ros::TimerEvent& event)
{
  if (g_first)
  {
    g_first = false;
    return;
  }

  double dt = (event.current_real - event.last_real).toSec();

  g_hero.update(dt);
  updateEnemies(dt);
  updateProjectiles(dt);
  updateExplosions(dt);
  g_marker_array_pub.publish(g_markers);
}

void commandCallback(const HeroCommandConstPtr& cmd)
{
  g_cmd = cmd;
}

void init(ros::NodeHandle& nh)
{
  g_marker_pub = nh.advertise<Marker>("shmup", 0);
  g_marker_array_pub = nh.advertise<MarkerArray>("shmup_array", 1);
  g_update_timer = nh.createTimer(ros::Duration(0.016), update);
  g_command_sub = nh.subscribe("shmup_command", 1, commandCallback);
  g_cmd.reset(new HeroCommand);

  initPlayingField();
  initHero();
  initEnemies();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_shmup");
  ros::NodeHandle nh;

  srand(time(NULL));

  init(nh);

  ros::spin();
}
