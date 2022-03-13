/*
 * Copyright 2008 Advanced Micro Devices, Inc.
 * Copyright 2008 Red Hat Inc.
 * Copyright 2009 Jerome Glisse.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Dave Airlie
 *          Alex Deucher
 *          Jerome Glisse
 *          Christian König
 */

#include <drm/drm_debugfs.h>
#include <drm/drm_file.h>

#include "radeon.h"

#include "evergreend.h"
/*
 * IB
 * IBs (Indirect Buffers) and areas of GPU accessible memory where
 * commands are stored.  You can put a pointer to the IB in the
 * command ring and the hw will fetch the commands from the IB
 * and execute them.  Generally userspace acceleration drivers
 * produce command buffers which are send to the kernel and
 * put in IBs for execution by the requested ring.
 */
static int radeon_debugfs_sa_init(struct radeon_device *rdev);

/**
 * radeon_ib_get - request an IB (Indirect Buffer)
 *
 * @rdev: radeon_device pointer
 * @ring: ring index the IB is associated with
 * @ib: IB object returned
 * @size: requested IB size
 *
 * Request an IB (all asics).  IBs are allocated using the
 * suballocator.
 * Returns 0 on success, error on failure.
 */
int radeon_ib_get(struct radeon_device *rdev, int ring,
		  struct radeon_ib *ib, struct radeon_vm *vm,
		  unsigned size)
{
	int r;

	r = radeon_sa_bo_new(rdev, &rdev->ring_tmp_bo, &ib->sa_bo, size, 256);
	if (r) {
		dev_err(rdev->dev, "failed to get a new IB (%d)\n", r);
		return r;
	}

	radeon_sync_create(&ib->sync);

	ib->ring = ring;
	ib->fence = NULL;
	ib->ptr = radeon_sa_bo_cpu_addr(ib->sa_bo);
	ib->vm = vm;
	if (vm) {
		/* ib pool is bound at RADEON_VA_IB_OFFSET in virtual address
		 * space and soffset is the offset inside the pool bo
		 */
		ib->gpu_addr = ib->sa_bo->soffset + RADEON_VA_IB_OFFSET;
	} else {
		ib->gpu_addr = radeon_sa_bo_gpu_addr(ib->sa_bo);
	}
	ib->is_const_ib = false;

	return 0;
}

/**
 * radeon_ib_free - free an IB (Indirect Buffer)
 *
 * @rdev: radeon_device pointer
 * @ib: IB object to free
 *
 * Free an IB (all asics).
 */
void radeon_ib_free(struct radeon_device *rdev, struct radeon_ib *ib)
{
	radeon_sync_free(rdev, &ib->sync, ib->fence);
	radeon_sa_bo_free(rdev, &ib->sa_bo, ib->fence);
	radeon_fence_unref(&ib->fence);
}

/**
 * radeon_ib_schedule - schedule an IB (Indirect Buffer) on the ring
 *
 * @rdev: radeon_device pointer
 * @ib: IB object to schedule
 * @const_ib: Const IB to schedule (SI only)
 * @hdp_flush: Whether or not to perform an HDP cache flush
 *
 * Schedule an IB on the associated ring (all asics).
 * Returns 0 on success, error on failure.
 *
 * On SI, there are two parallel engines fed from the primary ring,
 * the CE (Constant Engine) and the DE (Drawing Engine).  Since
 * resource descriptors have moved to memory, the CE allows you to
 * prime the caches while the DE is updating register state so that
 * the resource descriptors will be already in cache when the draw is
 * processed.  To accomplish this, the userspace driver submits two
 * IBs, one for the CE and one for the DE.  If there is a CE IB (called
 * a CONST_IB), it will be put on the ring prior to the DE IB.  Prior
 * to SI there was just a DE IB.
 */
int radeon_ib_schedule(struct radeon_device *rdev, struct radeon_ib *ib,
		       struct radeon_ib *const_ib, bool hdp_flush)
{
	struct radeon_ring *ring = &rdev->ring[ib->ring];
	int r = 0;
	int draw = rdev->numFSuses;



	printk("IB scheduled, dumping %d DWORDs\n",ib->length_dw);
	int i;
	int fb = 0;
	/*for(i = 0; i < ib->length_dw; i++){
		//printk("0x%x\n",ib->ptr[i]);
		if(ib->ptr[i] == 0xc0002f00){ // num_instances
			fb = 1; // prevents test image from being copied during the ring tests and resulting in a timeout. Triggering this via debugfs would probably be better, but whatever
			//ib->ptr[i] = 0xc0001000;  // change to nop
		}
		if(ib->ptr[i] == 0xc0012d00 && draw == 0){ // draw_index_auto
			ib->ptr[i] = 0xc0011000;  // change to nop
			printk("FS start address hasn't been used yet, likely contains garbage. Patching out draw to avoid crash\n");
			//ib->ptr[i+1] = 0x0; // change index_count to 0
			//ib->ptr[i+2] = 0x42;// opaque draw and auto-increment index

		}
		if(ib->ptr[i] == 0xc0016900){
			// set context reg
			if(ib->ptr[i+1] == 0x217){
				// SQ:SQ_PGM_START_VS
				uint64_t startaddr = ib->ptr[i+2];
				startaddr = startaddr << 8;	// address is saved 256 byte aligned
				printk("VS start address: 0x%llX vram start: 0x%llX gtt start: 0x%llX\n",startaddr,rdev->mc.vram_start,rdev->mc.gtt_start);

				int maxDwords = 200;	// max dwords read before aborting to not end up in an infinite loop
				int dw;
				printk("Dumping VS bytecode\n");
				uint32_t shader[maxDwords + 1];
				for(dw = 0; dw < maxDwords; dw++){
					radeon_ring_lock(rdev,ring,7);
						radeon_ring_write(ring,PACKET3(PACKET3_CP_DMA,4));
						radeon_ring_write(ring,lower_32_bits(startaddr + (dw * 4)));
						radeon_ring_write(ring,upper_32_bits(startaddr + (dw * 4)) & 0xFF);
						radeon_ring_write(ring,lower_32_bits(rdev->shader_read_gpu));
						radeon_ring_write(ring,upper_32_bits(rdev->shader_read_gpu) & 0xFF);
						radeon_ring_write(ring,((4) & 0xFFFFF));

						radeon_ring_unlock_commit(rdev,ring,false);
						udelay(10);
						uint32_t dword = readb(rdev->shader_read_cpu);
						dword |= readb(rdev->shader_read_cpu + 1) << 8;
						dword |= readb(rdev->shader_read_cpu + 2) << 16;
						dword |= readb(rdev->shader_read_cpu + 3) << 24;
						//printk("DWORD %d: 0x%X\n",dw,dword);
						shader[dw] = dword;
						if(dword == 0x95200688){
							printk("done taking a dump\n");
							break;
						}
						if(dword == 0x88062095){
							printk("done throwing up\n");
							break;
						}
				}
				int nr;
				for(nr = 0; nr < dw+1; nr += 2){
					//printk("0x%X: 0x%X 0x%X",nr,shader[nr],shader[nr + 1]);
				}
			}

			if(ib->ptr[i+1] == 0x210){
							// SQ:SQ_PGM_START_PS
							uint64_t startaddr = ib->ptr[i+2];
							startaddr = startaddr << 8;	// address is saved 256 byte aligned
							printk("PS start address: 0x%llX vram start: 0x%llX gtt start: 0x%llX\n",startaddr,rdev->mc.vram_start,rdev->mc.gtt_start);

							int maxDwords = 200;	// max dwords read before aborting to not end up in an infinite loop
							int dw;
							printk("Dumping PS bytecode\n");
							uint32_t shader[maxDwords + 1];
							for(dw = 0; dw < maxDwords; dw++){
								radeon_ring_lock(rdev,ring,7);
									radeon_ring_write(ring,PACKET3(PACKET3_CP_DMA,4));
									radeon_ring_write(ring,lower_32_bits(startaddr + (dw * 4)));
									radeon_ring_write(ring,upper_32_bits(startaddr + (dw * 4)) & 0xFF);
									radeon_ring_write(ring,lower_32_bits(rdev->shader_read_gpu));
									radeon_ring_write(ring,upper_32_bits(rdev->shader_read_gpu) & 0xFF);
									radeon_ring_write(ring,((4) & 0xFFFFF));

									radeon_ring_unlock_commit(rdev,ring,false);
									udelay(10);
									uint32_t dword = readl(rdev->shader_read_cpu);
									//printk("DWORD %d: 0x%X\n",dw,dword);
									shader[dw] = dword;
									if(dword == 0x95200688){
										printk("done taking a dump\n");
										break;
									}
									if(dword == 0x88062095){
										printk("done throwing up\n");
										break;
									}
							}
							int nr;
							for(nr = 0; nr < dw+1; nr += 2){
								//printk("0x%X: 0x%X 0x%X",nr,shader[nr],shader[nr + 1]);
							}
						}
			if(ib->ptr[i+1] == 0x229){
							// SQ:SQ_PGM_START_FS
							uint64_t startaddr = ib->ptr[i+2];
							startaddr = startaddr << 8;	// address is saved 256 byte aligned
							printk("FS start address: 0x%llX vram start: 0x%llX gtt start: 0x%llX\n",startaddr,rdev->mc.vram_start,rdev->mc.gtt_start);

							int maxDwords = 200;	// max dwords read before aborting to not end up in an infinite loop
							int dw;
							printk("Dumping FS bytecode\n");
							uint32_t shader[maxDwords + 1];
							for(dw = 0; dw < maxDwords; dw++){
								radeon_ring_lock(rdev,ring,7);
									radeon_ring_write(ring,PACKET3(PACKET3_CP_DMA,4));
									radeon_ring_write(ring,lower_32_bits(startaddr + (dw * 4)));
									radeon_ring_write(ring,upper_32_bits(startaddr + (dw * 4)) & 0xFF);
									radeon_ring_write(ring,lower_32_bits(rdev->shader_read_gpu));
									radeon_ring_write(ring,upper_32_bits(rdev->shader_read_gpu) & 0xFF);
									radeon_ring_write(ring,((4) & 0xFFFFF));

									radeon_ring_unlock_commit(rdev,ring,false);
									udelay(10);
									uint32_t dword = readl(rdev->shader_read_cpu);
									//printk("DWORD %d: 0x%X\n",dw,dword);
									shader[dw] = dword;
									if(dword == 0x95200688){
										printk("done taking a dump\n");
										break;
									}
									if(dword == 0x88062095){
										printk("done throwing up\n");
										break;
									}
							}
							int nr;
							for(nr = 0; nr < dw+1; nr += 2){
								//printk("0x%X: 0x%X 0x%X",nr,shader[nr],shader[nr + 1]);
							}
							/*printk("copying FS code\n");
							// first DWord
							writel(0x95200688,rdev->shader_read_cpu);
							radeon_ring_lock(rdev,ring,7);
							radeon_ring_write(ring,PACKET3(PACKET3_CP_DMA,4));
							radeon_ring_write(ring,lower_32_bits(rdev->shader_read_gpu));
							radeon_ring_write(ring,upper_32_bits(rdev->shader_read_gpu) & 0xFF);
							radeon_ring_write(ring,lower_32_bits(startaddr));
							radeon_ring_write(ring,upper_32_bits(startaddr) & 0xFF);
							radeon_ring_write(ring,((4) & 0xFFFFF));
							radeon_ring_unlock_commit(rdev,ring,false);
							udelay(10);
							// second DWord
							writel(0x0,rdev->shader_read_cpu);
							radeon_ring_lock(rdev,ring,7);
							radeon_ring_write(ring,PACKET3(PACKET3_CP_DMA,4));
							radeon_ring_write(ring,lower_32_bits(rdev->shader_read_gpu));
							radeon_ring_write(ring,upper_32_bits(rdev->shader_read_gpu) & 0xFF);
							radeon_ring_write(ring,lower_32_bits(startaddr+4));
							radeon_ring_write(ring,upper_32_bits(startaddr+4) & 0xFF);
							radeon_ring_write(ring,((4) & 0xFFFFF));
							radeon_ring_unlock_commit(rdev,ring,false);
							udelay(10);*/
							/*rdev->numFSuses++;
						}
		}
	}
	printk("patching out num_instances and draw_index_auto\n");*/

	//DMA test image to FB
	if(0){
		radeon_ring_lock(rdev,ring,7);
		radeon_ring_write(ring,PACKET3(PACKET3_CP_DMA,4));
		radeon_ring_write(ring,lower_32_bits(rdev->rick_gpu));
		radeon_ring_write(ring,upper_32_bits(rdev->rick_gpu) & 0xFF);
		radeon_ring_write(ring,lower_32_bits(rdev->fb_gpu));
		radeon_ring_write(ring,upper_32_bits(rdev->fb_gpu) & 0xFF);
		radeon_ring_write(ring,(1920*1080*4) & 0xFFFFF);

		radeon_ring_unlock_commit(rdev,ring,false);
		printk("DMAd test image to FB\n");
	}
	radeon_gart_sync_all_for_device(rdev);
	if (!ib->length_dw || !ring->ready) {
		/* TODO: Nothings in the ib we should report. */
		dev_err(rdev->dev, "couldn't schedule ib\n");
		return -EINVAL;
	}

	/* 64 dwords should be enough for fence too */
	r = radeon_ring_lock(rdev, ring, 64 + RADEON_NUM_SYNCS * 8);
	if (r) {
		dev_err(rdev->dev, "scheduling IB failed (%d).\n", r);
		return r;
	}

	/* grab a vm id if necessary */
	if (ib->vm) {
		struct radeon_fence *vm_id_fence;
		vm_id_fence = radeon_vm_grab_id(rdev, ib->vm, ib->ring);
		radeon_sync_fence(&ib->sync, vm_id_fence);
	}

	/* sync with other rings */
	r = radeon_sync_rings(rdev, &ib->sync, ib->ring);
	if (r) {
		dev_err(rdev->dev, "failed to sync rings (%d)\n", r);
		radeon_ring_unlock_undo(rdev, ring);
		return r;
	}

	if (ib->vm)
		radeon_vm_flush(rdev, ib->vm, ib->ring,
				ib->sync.last_vm_update);

	if (const_ib) {
		radeon_ring_ib_execute(rdev, const_ib->ring, const_ib);
		radeon_sync_free(rdev, &const_ib->sync, NULL);
	}
	radeon_ring_ib_execute(rdev, ib->ring, ib);
	r = radeon_fence_emit(rdev, &ib->fence, ib->ring);
	if (r) {
		dev_err(rdev->dev, "failed to emit fence for new IB (%d)\n", r);
		radeon_ring_unlock_undo(rdev, ring);
		return r;
	}
	if (const_ib) {
		const_ib->fence = radeon_fence_ref(ib->fence);
	}

	if (ib->vm)
		radeon_vm_fence(rdev, ib->vm, ib->fence);

	radeon_ring_unlock_commit(rdev, ring, hdp_flush);
	return 0;
}

/**
 * radeon_ib_pool_init - Init the IB (Indirect Buffer) pool
 *
 * @rdev: radeon_device pointer
 *
 * Initialize the suballocator to manage a pool of memory
 * for use as IBs (all asics).
 * Returns 0 on success, error on failure.
 */
int radeon_ib_pool_init(struct radeon_device *rdev)
{
	int r;

	if (rdev->ib_pool_ready) {
		return 0;
	}

	if (rdev->family >= CHIP_BONAIRE) {
		r = radeon_sa_bo_manager_init(rdev, &rdev->ring_tmp_bo,
					      RADEON_IB_POOL_SIZE*64*1024,
					      RADEON_GPU_PAGE_SIZE,
					      RADEON_GEM_DOMAIN_GTT,
					      RADEON_GEM_GTT_WC);
	} else {
		/* Before CIK, it's better to stick to cacheable GTT due
		 * to the command stream checking
		 */
		r = radeon_sa_bo_manager_init(rdev, &rdev->ring_tmp_bo,
					      RADEON_IB_POOL_SIZE*64*1024,
					      RADEON_GPU_PAGE_SIZE,
					      RADEON_GEM_DOMAIN_GTT,RADEON_GEM_GTT_UC);
	}
	if (r) {
		printk("error after radeon_sa_bo_manager_init: r=%d\n",r);
		return r;
	}

	r = radeon_sa_bo_manager_start(rdev, &rdev->ring_tmp_bo);
	if (r) {
		printk("error after radeon_sa_bo_manager_start: r=%d\n",r);
		return r;
	}

	rdev->ib_pool_ready = true;
	if (radeon_debugfs_sa_init(rdev)) {
		dev_err(rdev->dev, "failed to register debugfs file for SA\n");
	}
	return 0;
}

/**
 * radeon_ib_pool_fini - Free the IB (Indirect Buffer) pool
 *
 * @rdev: radeon_device pointer
 *
 * Tear down the suballocator managing the pool of memory
 * for use as IBs (all asics).
 */
void radeon_ib_pool_fini(struct radeon_device *rdev)
{
	if (rdev->ib_pool_ready) {
		radeon_sa_bo_manager_suspend(rdev, &rdev->ring_tmp_bo);
		radeon_sa_bo_manager_fini(rdev, &rdev->ring_tmp_bo);
		rdev->ib_pool_ready = false;
	}
}

/**
 * radeon_ib_ring_tests - test IBs on the rings
 *
 * @rdev: radeon_device pointer
 *
 * Test an IB (Indirect Buffer) on each ring.
 * If the test fails, disable the ring.
 * Returns 0 on success, error if the primary GFX ring
 * IB test fails.
 */
int radeon_ib_ring_tests(struct radeon_device *rdev)
{
	unsigned i;
	int r;

	for (i = 0; i < RADEON_NUM_RINGS; ++i) {
		struct radeon_ring *ring = &rdev->ring[i];

		if (!ring->ready)
			continue;

		r = radeon_ib_test(rdev, i, ring);
		if (r) {
			radeon_fence_driver_force_completion(rdev, i);
			ring->ready = false;
			rdev->needs_reset = false;

			if (i == RADEON_RING_TYPE_GFX_INDEX) {
				/* oh, oh, that's really bad */
				DRM_ERROR("radeon: failed testing IB on GFX ring (%d).\n", r);
				rdev->accel_working = false;
				return r;

			} else {
				/* still not good, but we can live with it */
				DRM_ERROR("radeon: failed testing IB on ring %d (%d).\n", i, r);
			}
		}
	}
	return 0;
}

/*
 * Debugfs info
 */
#if defined(CONFIG_DEBUG_FS)

static int radeon_debugfs_sa_info(struct seq_file *m, void *data)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct radeon_device *rdev = dev->dev_private;

	radeon_sa_bo_dump_debug_info(&rdev->ring_tmp_bo, m);

	return 0;

}

static struct drm_info_list radeon_debugfs_sa_list[] = {
	{"radeon_sa_info", &radeon_debugfs_sa_info, 0, NULL},
};

#endif

static int radeon_debugfs_sa_init(struct radeon_device *rdev)
{
#if defined(CONFIG_DEBUG_FS)
	return radeon_debugfs_add_files(rdev, radeon_debugfs_sa_list, 1);
#else
	return 0;
#endif
}
