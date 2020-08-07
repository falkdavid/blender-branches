/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2016 by Mike Erwin.
 * All rights reserved.
 */

/** \file
 * \ingroup gpu
 *
 * Manage GL vertex array IDs in a thread-safe way
 * Use these instead of glGenBuffers & its friends
 * - alloc must be called from a thread that is bound
 *   to the context that will be used for drawing with
 *   this vao.
 * - free can be called from any thread
 */

/* TODO Create cmake option. */
#define WITH_OPENGL_BACKEND 1

#include "BLI_assert.h"
#include "BLI_utildefines.h"

#include "GPU_context.h"
#include "GPU_framebuffer.h"

#include "GHOST_C-api.h"

#include "gpu_backend.hh"
#include "gpu_batch_private.h"
#include "gpu_context_private.h"
#include "gpu_matrix_private.h"

#ifdef WITH_OPENGL_BACKEND
#  include "gl_backend.hh"
#  include "gl_context.hh"
#endif

#include <mutex>
#include <vector>

static std::vector<GLuint> orphaned_buffer_ids;
static std::vector<GLuint> orphaned_texture_ids;

static std::mutex orphans_mutex;
static std::mutex main_context_mutex;

static thread_local GPUContext *active_ctx = NULL;

static void orphans_add(GPUContext *ctx, std::vector<GLuint> *orphan_list, GLuint id)
{
  std::mutex *mutex = (ctx) ? &ctx->orphans_mutex : &orphans_mutex;

  mutex->lock();
  orphan_list->emplace_back(id);
  mutex->unlock();
}

static void orphans_clear(GPUContext *ctx)
{
  /* need at least an active context */
  BLI_assert(ctx);

  /* context has been activated by another thread! */
  BLI_assert(pthread_equal(pthread_self(), ctx->thread));

  ctx->orphans_mutex.lock();
  if (!ctx->orphaned_vertarray_ids.empty()) {
    uint orphan_len = (uint)ctx->orphaned_vertarray_ids.size();
    glDeleteVertexArrays(orphan_len, ctx->orphaned_vertarray_ids.data());
    ctx->orphaned_vertarray_ids.clear();
  }
  if (!ctx->orphaned_framebuffer_ids.empty()) {
    uint orphan_len = (uint)ctx->orphaned_framebuffer_ids.size();
    glDeleteFramebuffers(orphan_len, ctx->orphaned_framebuffer_ids.data());
    ctx->orphaned_framebuffer_ids.clear();
  }

  ctx->orphans_mutex.unlock();

  orphans_mutex.lock();
  if (!orphaned_buffer_ids.empty()) {
    uint orphan_len = (uint)orphaned_buffer_ids.size();
    glDeleteBuffers(orphan_len, orphaned_buffer_ids.data());
    orphaned_buffer_ids.clear();
  }
  if (!orphaned_texture_ids.empty()) {
    uint orphan_len = (uint)orphaned_texture_ids.size();
    glDeleteTextures(orphan_len, orphaned_texture_ids.data());
    orphaned_texture_ids.clear();
  }
  orphans_mutex.unlock();
}

GPUContext *GPU_context_create(void *ghost_window)
{
  if (gpu_backend_get() == NULL) {
    /* TODO move where it make sense. */
    GPU_backend_init(GPU_BACKEND_OPENGL);
  }

  GPUContext *ctx = gpu_backend_get()->context_alloc(ghost_window);
  glGenVertexArrays(1, &ctx->default_vao);
  if (ghost_window != NULL) {
    ctx->default_framebuffer = GHOST_GetDefaultOpenGLFramebuffer((GHOST_WindowHandle)ghost_window);
  }
  else {
    ctx->default_framebuffer = 0;
  }

  ctx->matrix_state = GPU_matrix_state_create();
  GPU_context_active_set(ctx);
  return ctx;
}

/* to be called after GPU_context_active_set(ctx_to_destroy) */
void GPU_context_discard(GPUContext *ctx)
{
  /* Make sure no other thread has locked it. */
  BLI_assert(ctx == active_ctx);
  BLI_assert(pthread_equal(pthread_self(), ctx->thread));
  BLI_assert(ctx->orphaned_vertarray_ids.empty());
#ifdef DEBUG
  /* For now don't allow GPUFrameBuffers to be reuse in another ctx. */
  BLI_assert(ctx->framebuffers.empty());
#endif
  /* delete remaining vaos */
  while (!ctx->batches.empty()) {
    /* this removes the array entry */
    GPU_batch_vao_cache_clear(*ctx->batches.begin());
  }
  GPU_matrix_state_discard(ctx->matrix_state);
  glDeleteVertexArrays(1, &ctx->default_vao);
  delete ctx;
  active_ctx = NULL;
}

/* ctx can be NULL */
void GPU_context_active_set(GPUContext *ctx)
{
#if TRUST_NO_ONE
  if (active_ctx) {
    active_ctx->thread_is_used = false;
  }
  /* Make sure no other context is already bound to this thread. */
  if (ctx) {
    /* Make sure no other thread has locked it. */
    assert(ctx->thread_is_used == false);
    ctx->thread = pthread_self();
    ctx->thread_is_used = true;
  }
#endif
  if (ctx) {
    orphans_clear(ctx);
  }
  active_ctx = ctx;
}

GPUContext *GPU_context_active_get(void)
{
  return active_ctx;
}

GLuint GPU_vao_default(void)
{
  BLI_assert(active_ctx); /* need at least an active context */
  BLI_assert(pthread_equal(
      pthread_self(), active_ctx->thread)); /* context has been activated by another thread! */
  return active_ctx->default_vao;
}

GLuint GPU_framebuffer_default(void)
{
  BLI_assert(active_ctx); /* need at least an active context */
  BLI_assert(pthread_equal(
      pthread_self(), active_ctx->thread)); /* context has been activated by another thread! */
  return active_ctx->default_framebuffer;
}

GLuint GPU_vao_alloc(void)
{
  GLuint new_vao_id = 0;
  orphans_clear(active_ctx);
  glGenVertexArrays(1, &new_vao_id);
  return new_vao_id;
}

GLuint GPU_fbo_alloc(void)
{
  GLuint new_fbo_id = 0;
  orphans_clear(active_ctx);
  glGenFramebuffers(1, &new_fbo_id);
  return new_fbo_id;
}

GLuint GPU_buf_alloc(void)
{
  GLuint new_buffer_id = 0;
  orphans_clear(active_ctx);
  glGenBuffers(1, &new_buffer_id);
  return new_buffer_id;
}

GLuint GPU_tex_alloc(void)
{
  GLuint new_texture_id = 0;
  orphans_clear(active_ctx);
  glGenTextures(1, &new_texture_id);
  return new_texture_id;
}

void GPU_vao_free(GLuint vao_id, GPUContext *ctx)
{
  BLI_assert(ctx);
  if (ctx == active_ctx) {
    glDeleteVertexArrays(1, &vao_id);
  }
  else {
    orphans_add(ctx, &ctx->orphaned_vertarray_ids, vao_id);
  }
}

void GPU_fbo_free(GLuint fbo_id, GPUContext *ctx)
{
  BLI_assert(ctx);
  if (ctx == active_ctx) {
    glDeleteFramebuffers(1, &fbo_id);
  }
  else {
    orphans_add(ctx, &ctx->orphaned_framebuffer_ids, fbo_id);
  }
}

void GPU_buf_free(GLuint buf_id)
{
  if (active_ctx) {
    glDeleteBuffers(1, &buf_id);
  }
  else {
    orphans_add(NULL, &orphaned_buffer_ids, buf_id);
  }
}

void GPU_tex_free(GLuint tex_id)
{
  if (active_ctx) {
    glDeleteTextures(1, &tex_id);
  }
  else {
    orphans_add(NULL, &orphaned_texture_ids, tex_id);
  }
}

/* GPUBatch & GPUFrameBuffer contains respectively VAO & FBO indices
 * which are not shared across contexts. So we need to keep track of
 * ownership. */

void gpu_context_add_batch(GPUContext *ctx, GPUBatch *batch)
{
  BLI_assert(ctx);
  ctx->orphans_mutex.lock();
  ctx->batches.emplace(batch);
  ctx->orphans_mutex.unlock();
}

void gpu_context_remove_batch(GPUContext *ctx, GPUBatch *batch)
{
  BLI_assert(ctx);
  ctx->orphans_mutex.lock();
  ctx->batches.erase(batch);
  ctx->orphans_mutex.unlock();
}

void gpu_context_add_framebuffer(GPUContext *ctx, GPUFrameBuffer *fb)
{
#ifdef DEBUG
  BLI_assert(ctx);
  ctx->orphans_mutex.lock();
  ctx->framebuffers.emplace(fb);
  ctx->orphans_mutex.unlock();
#else
  UNUSED_VARS(ctx, fb);
#endif
}

void gpu_context_remove_framebuffer(GPUContext *ctx, GPUFrameBuffer *fb)
{
#ifdef DEBUG
  BLI_assert(ctx);
  ctx->orphans_mutex.lock();
  ctx->framebuffers.erase(fb);
  ctx->orphans_mutex.unlock();
#else
  UNUSED_VARS(ctx, fb);
#endif
}

void gpu_context_active_framebuffer_set(GPUContext *ctx, GPUFrameBuffer *fb)
{
  ctx->current_fbo = fb;
}

GPUFrameBuffer *gpu_context_active_framebuffer_get(GPUContext *ctx)
{
  return ctx->current_fbo;
}

struct GPUMatrixState *gpu_context_active_matrix_state_get()
{
  BLI_assert(active_ctx);
  return active_ctx->matrix_state;
}

void GPU_context_main_lock(void)
{
  main_context_mutex.lock();
}

void GPU_context_main_unlock(void)
{
  main_context_mutex.unlock();
}

/* -------------------------------------------------------------------- */
/** \name Backend selection
 * \{ */

static GPUBackend *g_backend;

void GPU_backend_init(eGPUBackendType backend_type)
{
  BLI_assert(g_backend == NULL);

  switch (backend_type) {
#if WITH_OPENGL_BACKEND
    case GPU_BACKEND_OPENGL:
      g_backend = new GLBackend;
      break;
#endif
    default:
      BLI_assert(0);
      break;
  }
}

void GPU_backend_exit(void)
{
  delete g_backend;
}

GPUBackend *gpu_backend_get(void)
{
  return g_backend;
}

/** \} */
