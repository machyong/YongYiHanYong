class MLP(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : NoneType
  __annotations__["0"] = __torch__.torch.nn.modules.linear.Linear
  __annotations__["1"] = __torch__.torch.nn.modules.activation.ELU
  __annotations__["2"] = __torch__.torch.nn.modules.linear.___torch_mangle_3.Linear
  __annotations__["3"] = __torch__.torch.nn.modules.activation.ELU
  __annotations__["4"] = __torch__.torch.nn.modules.linear.___torch_mangle_4.Linear
  __annotations__["5"] = __torch__.torch.nn.modules.activation.ELU
  __annotations__["6"] = __torch__.torch.nn.modules.linear.___torch_mangle_5.Linear
  def forward(self: __torch__.rsl_rl.networks.mlp.MLP,
    x: Tensor) -> Tensor:
    _0 = getattr(self, "0")
    _1 = getattr(self, "1")
    _2 = getattr(self, "2")
    _3 = getattr(self, "3")
    _4 = getattr(self, "4")
    _5 = getattr(self, "5")
    _6 = getattr(self, "6")
    x0 = (_0).forward(x, )
    x1 = (_1).forward(x0, )
    x2 = (_2).forward(x1, )
    x3 = (_3).forward(x2, )
    x4 = (_4).forward(x3, )
    x5 = (_5).forward(x4, )
    return (_6).forward(x5, )
  def __len__(self: __torch__.rsl_rl.networks.mlp.MLP) -> int:
    return 7
