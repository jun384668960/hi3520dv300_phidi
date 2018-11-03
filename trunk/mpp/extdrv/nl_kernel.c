/*******************************
file:           k_netlink.c
description:    netlink demo
kernel:         linux-3.10.36
author:         arvik
email:          1216601195@qq.com
blog:           http://blog.csdn.net/u012819339
*******************************/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/kmemcheck.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/inet.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#ifdef CONFIG_NET_CLS_ACT
#include <net/pkt_sched.h>
#endif
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/splice.h>
#include <linux/cache.h>
#include <linux/rtnetlink.h>
#include <linux/init.h>
#include <linux/scatterlist.h>
#include <linux/errqueue.h>
#include <linux/prefetch.h>

#include <net/protocol.h>
#include <net/dst.h>
#include <net/sock.h>
#include <net/checksum.h>
#include <net/xfrm.h>

#include <asm/uaccess.h>
#include <trace/events/skb.h>
#include <linux/highmem.h>


#define NETLINK_USER  23
#define USER_MSG    (NETLINK_USER + 1)
#define USER_PORT   50

MODULE_LICENSE("GPL");  
MODULE_AUTHOR("arvik");  
MODULE_DESCRIPTION("netlink_demo");

static struct sock *netlinkfd = NULL;
#if 0
struct kmem_cache *skbuff_head_cache __read_mostly;
static struct kmem_cache *skbuff_fclone_cache __read_mostly;

#define kmalloc_reserve(size, gfp, node, pfmemalloc) \
	 __kmalloc_reserve(size, gfp, node, _RET_IP_, pfmemalloc)

static void *__kmalloc_reserve(size_t size, gfp_t flags, int node,
			       unsigned long ip, bool *pfmemalloc)
{
	void *obj;
	bool ret_pfmemalloc = false;

	/*
	 * Try a regular allocation, when that fails and we're not entitled
	 * to the reserves, fail.
	 */
	obj = kmalloc_node_track_caller(size,
					flags | __GFP_NOMEMALLOC | __GFP_NOWARN,
					node);
	if (obj || !(gfp_pfmemalloc_allowed(flags)))
		goto out;

	/* Try again but now we are using pfmemalloc reserves */
	ret_pfmemalloc = true;
	obj = kmalloc_node_track_caller(size, flags, node);

out:
	if (pfmemalloc)
		*pfmemalloc = ret_pfmemalloc;

	return obj;
}


struct sk_buff *my_alloc_skb(unsigned int size, gfp_t gfp_mask,
			    int flags, int node)
{
	struct kmem_cache *cache;
	struct skb_shared_info *shinfo;
	struct sk_buff *skb;
	u8 *data;
	bool pfmemalloc;

	cache = (flags & SKB_ALLOC_FCLONE)
		? skbuff_fclone_cache : skbuff_head_cache;

	if (sk_memalloc_socks() && (flags & SKB_ALLOC_RX))
		gfp_mask |= __GFP_MEMALLOC;

	/* Get the HEAD */
	skb = kmem_cache_alloc_node(cache, gfp_mask & ~__GFP_DMA, node);
	if (!skb)
		goto out;
	prefetchw(skb);

	/* We do our best to align skb_shared_info on a separate cache
	 * line. It usually works because kmalloc(X > SMP_CACHE_BYTES) gives
	 * aligned memory blocks, unless SLUB/SLAB debug is enabled.
	 * Both skb->head and skb_shared_info are cache line aligned.
	 */
	size = SKB_DATA_ALIGN(size);
	size += SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
	data = kmalloc_reserve(size, gfp_mask, node, &pfmemalloc);
	if (!data)
		goto nodata;
	/* kmalloc(size) might give us more room than requested.
	 * Put skb_shared_info exactly at the end of allocated zone,
	 * to allow max possible filling before reallocation.
	 */
	size = SKB_WITH_OVERHEAD(ksize(data));
	prefetchw(data + size);

	/*
	 * Only clear those fields we need to clear, not those that we will
	 * actually initialise below. Hence, don't put any more fields after
	 * the tail pointer in struct sk_buff!
	 */
	memset(skb, 0, offsetof(struct sk_buff, tail));
	/* Account for allocated memory : skb + skb->head */
	skb->truesize = SKB_TRUESIZE(size);
	skb->pfmemalloc = pfmemalloc;
	atomic_set(&skb->users, 1);
	skb->head = data;
	skb->data = data;
	skb_reset_tail_pointer(skb);
	skb->end = skb->tail + size;
#ifdef NET_SKBUFF_DATA_USES_OFFSET
	skb->mac_header = ~0U;
	skb->transport_header = ~0U;
#endif

	/* make sure we initialize shinfo sequentially */
	shinfo = skb_shinfo(skb);
	memset(shinfo, 0, offsetof(struct skb_shared_info, dataref));
	atomic_set(&shinfo->dataref, 1);
	kmemcheck_annotate_variable(shinfo->destructor_arg);

	if (flags & SKB_ALLOC_FCLONE) {
		struct sk_buff *child = skb + 1;
		atomic_t *fclone_ref = (atomic_t *) (child + 1);

		kmemcheck_annotate_bitfield(child, flags1);
		kmemcheck_annotate_bitfield(child, flags2);
		skb->fclone = SKB_FCLONE_ORIG;
		atomic_set(fclone_ref, 1);

		child->fclone = SKB_FCLONE_UNAVAILABLE;
		child->pfmemalloc = pfmemalloc;
	}
out:
	return skb;
nodata:
	kmem_cache_free(cache, skb);
	skb = NULL;
	goto out;
}
#endif
int send_msg(int8_t *pbuf, uint16_t len)
{
    struct sk_buff *nl_skb = NULL;
    struct nlmsghdr *nlh;
    int ret;

    nl_skb = nlmsg_new(len+100, GFP_KERNEL);
    if(!nl_skb)
    {
        printk("netlink_alloc_skb error\n");
        return -1;
    }

	printk("0x%p %d skb_tailroom(skb):%d %d end:%d tail:%d head:0x%x, data:0x%x skb->truesize:%d\n",
		nl_skb, skb_is_nonlinear(nl_skb), skb_tailroom(nl_skb), nlmsg_total_size(USER_MSG), nl_skb->end, nl_skb->tail, nl_skb->head, nl_skb->data,
		nl_skb->truesize);

    nlh = nlmsg_put(nl_skb, 0, 0, USER_MSG, len, 0);
    if(nlh == NULL)
    {
        printk("nlmsg_put() error\n");
        nlmsg_free(nl_skb);
        return -1;
    }
    memcpy(nlmsg_data(nlh), pbuf, len);

    ret = netlink_unicast(netlinkfd, nl_skb, USER_PORT, MSG_DONTWAIT);

    return ret;
}


static void recv_cb(struct sk_buff *skb)
{
    struct nlmsghdr *nlh = NULL;
    void *data = NULL;

    printk("skb->len:%u\n", skb->len);
	printk("0x%p %d skb_tailroom(skb):%d %d end:%d tail:%d\n",
		skb, skb_is_nonlinear(skb), skb_tailroom(skb), nlmsg_total_size(USER_MSG), skb->end, skb->tail);
    if(skb->len >= nlmsg_total_size(0))
    {
        nlh = nlmsg_hdr(skb);
        data = nlmsg_data(nlh);
        if(data)
        {
            printk("kernel receive data: %s\n", (int8_t *)data);
            send_msg(data, nlmsg_len(nlh));
        }
    }
} 


struct netlink_kernel_cfg cfg = 
{
    .input = recv_cb,
};

static int __init test_netlink_init(void)
{
    printk("init netlink_demo!\n");


    netlinkfd = netlink_kernel_create(&init_net, USER_MSG, &cfg);
    if(!netlinkfd)
    {
        printk(KERN_ERR "can not create a netlink socket!\n");
        return -1;
    }

    printk("netlink demo init ok!\n");
    return 0;
}

static void __exit test_netlink_exit(void)
{
    sock_release(netlinkfd->sk_socket);
    printk(KERN_DEBUG "netlink exit\n!");
}

module_init(test_netlink_init);
module_exit(test_netlink_exit);
