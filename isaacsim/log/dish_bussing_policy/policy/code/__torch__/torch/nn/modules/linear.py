class Linear(Module):
  __parameters__ = ["weight", "bias", ]
  __buffers__ = []
  weight : Tensor
  bias : Tensor
  training : bool
  _is_full_backward_hook : NoneType
  in_features : Final[int] = 25
  out_features : Final[int] = 256
  def forward(self: __torch__.torch.nn.modules.linear.Linear,
    input: Tensor) -> Tensor:
    weight = self.weight
    bias = self.bias
    return torch.linear(input, weight, bias)
class Identity(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  def forward(self: __torch__.torch.nn.modules.linear.Identity,
    input: Tensor) -> Tensor:
    return input
